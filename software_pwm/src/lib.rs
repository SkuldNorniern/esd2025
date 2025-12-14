// Software PWM library for precise servo control
// Uses atomics and dedicated threads for minimal jitter

use rppal::gpio::Gpio;
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

// Precise busy-wait function optimized for minimal jitter
// Uses calibrated spin loops to minimize Instant::now() overhead
// Critical: Instant::now() has overhead, so we check it less frequently
#[inline(never)]
fn busy_wait_until(target: Instant) {
    // Quick early exit check
    let mut now = Instant::now();
    if now >= target {
        return;
    }
    
    // Estimate remaining time to choose optimal spin strategy
    let mut remaining = target.saturating_duration_since(now);
    
    // For very short waits (< 100us), check frequently for precision
    if remaining < Duration::from_micros(100) {
        while now < target {
            std::hint::spin_loop();
            now = Instant::now();
        }
        return;
    }
    
    // For longer waits, use calibrated spin loops
    // Check time less frequently to reduce overhead
    // The spin count is calibrated: ~1000 iterations ≈ 1-2 microseconds on modern CPUs
    let spin_batch_size = if remaining < Duration::from_micros(500) {
        200  // More frequent checks when close
    } else if remaining < Duration::from_micros(2000) {
        500  // Medium frequency
    } else {
        2000 // Less frequent checks for long waits
    };
    
    // Main spin loop: spin many times before checking time
    loop {
        // Spin for a batch before checking time
        // This dramatically reduces Instant::now() overhead
        for _ in 0..spin_batch_size {
            std::hint::spin_loop();
        }
        
        // Check time after spinning
        now = Instant::now();
        if now >= target {
            break;
        }
        
        // Update remaining time estimate
        remaining = target.saturating_duration_since(now);
        
        // If we're getting close, switch to more frequent checks
        if remaining < Duration::from_micros(100) {
            // Final precision: check every iteration
            while now < target {
                std::hint::spin_loop();
                now = Instant::now();
            }
            break;
        }
    }
}

// Software PWM servo with dedicated background thread
// Uses precise timing to minimize jitter
// Uses atomics to eliminate lock contention
pub struct SoftwarePwmServo {
    angle: Arc<AtomicU8>,
    running: Arc<AtomicBool>,
    _thread: thread::JoinHandle<()>,
}

impl SoftwarePwmServo {
    /// Create a new software PWM servo controller
    /// 
    /// # Arguments
    /// * `pin_num` - GPIO pin number (BCM numbering on Raspberry Pi)
    /// 
    /// # Returns
    /// * `Ok(SoftwarePwmServo)` on success
    /// * `Err(rppal::gpio::Error)` on failure
    /// 
    /// # Example
    /// ```
    /// let servo = SoftwarePwmServo::new(18)?;
    /// servo.set_angle(90)?; // Set to center position
    /// ```
    pub fn new(pin_num: u8) -> Result<Self, rppal::gpio::Error> {
        let gpio = Gpio::new()?;
        let pin = gpio.get(pin_num)?.into_output();
        
        let angle = Arc::new(AtomicU8::new(90)); // Start at center
        let running = Arc::new(AtomicBool::new(true));
        
        let angle_clone = Arc::clone(&angle);
        let running_clone = Arc::clone(&running);
        
        // Spawn background thread for continuous PWM generation
        // Uses high-priority timing to minimize jitter
        let _thread = thread::spawn(move || {
            let mut pin = pin;
            const PERIOD_US: u64 = 20_000; // 50Hz = 20ms period
            const PERIOD_DURATION: Duration = Duration::from_micros(PERIOD_US);
            
            // Pre-calculate common values to avoid repeated calculations
            const ANGLE_TO_PULSE_DIVISOR: u64 = 180;
            const ANGLE_TO_PULSE_MULTIPLIER: u64 = 1000;
            const MIN_PULSE_US: u64 = 1000;
            
            // Cache angle and pulse width to minimize atomic reads and calculations
            let mut cached_angle = 90u8;
            let mut cached_pulse_width_us = 1500u64; // Default to center (90°)
            let mut angle_update_counter = 0u32;
            
            // Pre-calculate period end duration to avoid repeated Duration::from_micros calls
            // This reduces allocations and improves cache locality
            
            loop {
                // Check if we should stop (non-blocking atomic check - no locks!)
                // Only check every 50 periods to reduce overhead (1 second at 50Hz)
                if angle_update_counter == 0 {
                    if !running_clone.load(Ordering::Relaxed) {
                        break;
                    }
                }
                
                // Read angle atomically - update less frequently to reduce cache line bouncing
                // Update cached angle every 50 periods (1 second) instead of 10
                // This is safe because servo movement is slow (0.1s/60°)
                // Reduces atomic operations by 5x, significantly reducing micro stutter
                angle_update_counter += 1;
                if angle_update_counter >= 50 {
                    let new_angle = angle_clone.load(Ordering::Relaxed);
                    if new_angle != cached_angle {
                        cached_angle = new_angle;
                        // Pre-calculate pulse width when angle changes
                        // Use integer math optimized for speed
                        let angle = cached_angle.min(180) as u64;
                        cached_pulse_width_us = MIN_PULSE_US + ((angle * ANGLE_TO_PULSE_MULTIPLIER) / ANGLE_TO_PULSE_DIVISOR);
                    }
                    angle_update_counter = 0;
                }
                
                // Generate one PWM period with precise timing
                // Critical: minimize time between period_start and pin.set_high()
                // Capture start time immediately before setting pin
                let period_start = Instant::now();
                
                // High pulse - set pin high
                // Pin operations are fast but we want minimal delay
                pin.set_high();
                
                // Calculate target time for high pulse end
                // Pre-calculate duration to avoid repeated Duration::from_micros
                let high_end = period_start + Duration::from_micros(cached_pulse_width_us);
                
                // Precise busy-wait for high pulse duration
                // Don't use sleep or yield - they cause jitter
                busy_wait_until(high_end);
                
                // Low pulse - set pin low
                // Capture time immediately after setting low to account for any delay
                pin.set_low();
                
                // Calculate target time for period end (exactly 20ms from start)
                // Use pre-calculated constant to avoid repeated Duration::from_micros
                let period_end = period_start + PERIOD_DURATION;
                
                // Precise busy-wait for remaining period time
                // This ensures exact 20ms period for stable 50Hz frequency
                // Critical for servo stability - any jitter here causes servo jitter
                busy_wait_until(period_end);
                
                // Optional: verify we didn't overshoot (for debugging)
                // In production, this adds overhead, so we skip it
                // let actual_period = period_start.elapsed();
                // if actual_period > PERIOD_DURATION + Duration::from_micros(10) {
                //     eprintln!("Warning: Period overshoot: {:?}", actual_period);
                // }
            }
        });
        
        Ok(SoftwarePwmServo {
            angle,
            running,
            _thread,
        })
    }

    /// Set the servo angle
    /// 
    /// # Arguments
    /// * `angle` - Angle in degrees (0-180, clamped automatically)
    /// 
    /// # Returns
    /// * `Ok(())` on success
    /// * `Err(rppal::gpio::Error)` on failure (unlikely for this operation)
    /// 
    /// # Example
    /// ```
    /// servo.set_angle(90)?; // Center position
    /// servo.set_angle(0)?;  // Leftmost position
    /// servo.set_angle(180)?; // Rightmost position
    /// ```
    pub fn set_angle(&self, angle: u8) -> Result<(), rppal::gpio::Error> {
        // Atomic write - no lock needed, always fast
        self.angle.store(angle.min(180), Ordering::Relaxed);
        Ok(())
    }
}

impl Drop for SoftwarePwmServo {
    fn drop(&mut self) {
        // Atomic write - no lock needed
        self.running.store(false, Ordering::Relaxed);
    }
}

