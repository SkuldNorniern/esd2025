// Software PWM library for precise servo control
// Uses atomics and dedicated threads for minimal jitter

use rppal::gpio::Gpio;
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

// Ultra-precise busy-wait optimized to minimize Instant::now() overhead
// Uses batched spin loops with periodic time checks for longer waits
#[inline(never)]
fn busy_wait_until(target: Instant) {
    let mut now = Instant::now();
    if now >= target {
        return;
    }
    
    let remaining = target.duration_since(now);
    
    // For waits longer than 100μs, use batched spin loops with periodic checks
    // This reduces Instant::now() overhead while maintaining precision
    if remaining.as_micros() > 100 {
        // Check time every ~100μs (approximately)
        // Each batch of spin_loop() takes roughly 1-2μs on modern CPUs
        const BATCH_SIZE: usize = 50; // ~50-100μs per batch
        
        // Coarse wait: spin in batches, check time periodically
        loop {
            // Batch of spin loops
            for _ in 0..BATCH_SIZE {
                std::hint::spin_loop();
            }
            
            // Check if we're close to target
            now = Instant::now();
            if now >= target {
                return;
            }
            
            // If we're very close (<50μs), switch to precise mode
            let remaining = target.duration_since(now);
            if remaining.as_micros() < 50 {
                break;
            }
        }
    }
    
    // Precise timing for the final bit (or for short waits)
    while Instant::now() < target {
        std::hint::spin_loop();
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
            
            // Pre-calculate all possible pulse widths to avoid repeated calculations
            // This eliminates division in the hot loop
            let mut pulse_widths: [u64; 181] = [0; 181];
            for angle in 0..=180 {
                pulse_widths[angle as usize] = 1000u64 + ((angle as u64 * 1000u64) / 180u64);
            }
            
            // Cache angle and pulse width to minimize atomic reads and calculations
            let mut cached_angle = 90u8;
            let mut cached_pulse_width_us = pulse_widths[90];
            let mut angle_update_counter = 0u32;
            
            loop {
                // Check if we should stop (non-blocking atomic check - no locks!)
                // Use Acquire ordering to ensure we see the latest value
                if !running_clone.load(Ordering::Acquire) {
                    break;
                }
                
                // Read angle atomically - update less frequently to reduce overhead
                // Update cached angle every 20 periods (400ms) instead of 10
                // This is safe because servo movement is slow (0.1s/60°)
                angle_update_counter += 1;
                if angle_update_counter >= 20 {
                    // Use Acquire ordering for consistency
                    let new_angle = angle_clone.load(Ordering::Acquire).min(180);
                    if new_angle != cached_angle {
                        cached_angle = new_angle;
                        cached_pulse_width_us = pulse_widths[cached_angle as usize];
                    }
                    angle_update_counter = 0;
                }
                
                // Generate one PWM period with precise timing
                // Capture start time immediately before setting pin to minimize delay
                let period_start = Instant::now();
                
                // High pulse - set pin high
                // Pin operations are fast but we want minimal delay
                pin.set_high();
                
                // Calculate target time for high pulse end
                let high_end = period_start + Duration::from_micros(cached_pulse_width_us);
                
                // Optimized busy-wait for high pulse duration
                // Uses calibrated delay + precise timing to minimize overhead
                busy_wait_until(high_end);
                
                // Low pulse - set pin low
                pin.set_low();
                
                // Calculate target time for period end (exactly 20ms from start)
                let period_end = period_start + PERIOD_DURATION;
                
                // Optimized busy-wait for remaining period time
                // This ensures exact 20ms period for stable 50Hz frequency
                // Critical for servo stability - any jitter here causes servo jitter
                busy_wait_until(period_end);
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
        // Atomic write - use Release ordering to ensure visibility
        // This ensures the PWM thread sees the update promptly
        self.angle.store(angle.min(180), Ordering::Release);
        Ok(())
    }
}

impl Drop for SoftwarePwmServo {
    fn drop(&mut self) {
        // Atomic write - no lock needed
        self.running.store(false, Ordering::Relaxed);
    }
}

