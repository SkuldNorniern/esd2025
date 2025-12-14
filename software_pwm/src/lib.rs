// Software PWM library for precise servo control
// Uses atomics and dedicated threads for minimal jitter

use rppal::gpio::Gpio;
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

// Precise busy-wait function that doesn't yield to scheduler
// Uses CPU cycles for accurate microsecond-level timing
#[inline(never)]
fn busy_wait_until(target: Instant) {
    // Use a tight loop without yielding to maintain precise timing
    // This is acceptable in a dedicated PWM thread
    while Instant::now() < target {
        // Empty loop - compiler won't optimize this away due to Instant::now()
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
            
            // Cache angle to reduce atomic reads
            let mut cached_angle = 90u8;
            let mut angle_update_counter = 0u32;
            
            loop {
                // Check if we should stop (non-blocking atomic check - no locks!)
                if !running_clone.load(Ordering::Relaxed) {
                    break;
                }
                
                // Read angle atomically - no lock needed, always fast
                // Update cached angle every 10 periods to reduce atomic reads
                // This is safe because servo movement is slow (0.1s/60°)
                angle_update_counter += 1;
                if angle_update_counter >= 10 {
                    cached_angle = angle_clone.load(Ordering::Relaxed);
                    angle_update_counter = 0;
                }
                
                let angle = cached_angle.min(180) as u64;
                
                // Convert angle to pulse width in microseconds
                // 0° = 1000μs (1ms), 90° = 1500μs (1.5ms), 180° = 2000μs (2ms)
                // Linear interpolation: pulse = 1000 + (angle * 1000) / 180
                // Using integer math for speed
                let pulse_width_us = 1000u64 + ((angle * 1000u64) / 180u64);
                
                // Generate one PWM period with precise timing
                // Capture start time immediately before setting pin
                let period_start = Instant::now();
                
                // High pulse - set pin high
                // Pin operations are fast but we want minimal delay
                pin.set_high();
                
                // Calculate target time for high pulse end
                let high_end = period_start + Duration::from_micros(pulse_width_us);
                
                // Precise busy-wait for high pulse duration
                // Don't use sleep or yield - they cause jitter
                busy_wait_until(high_end);
                
                // Low pulse - set pin low
                pin.set_low();
                
                // Calculate target time for period end (exactly 20ms from start)
                let period_end = period_start + PERIOD_DURATION;
                
                // Precise busy-wait for remaining period time
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
