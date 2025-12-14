use rppal::gpio::Gpio;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

// MG90S servo specifications (from datasheet):
// - Operating frequency: 50Hz (20ms period)
// - Pulse width range: 1ms (-90°) to 2ms (+90°)
// - Neutral position: 1.5ms (0°)
// - Dead band width: 5μs
// - Operating speed: 0.1s/60° (4.8V), 0.08s/60° (6V)
// - Operating voltage: 4.8V-6.0V

// Software PWM servo with dedicated background thread
// Uses precise timing to minimize jitter
struct SoftwarePwmServo {
    angle: Arc<Mutex<u8>>,
    running: Arc<Mutex<bool>>,
    _thread: thread::JoinHandle<()>,
}

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

impl SoftwarePwmServo {
    fn new(pin_num: u8) -> Result<Self, rppal::gpio::Error> {
        let gpio = Gpio::new()?;
        let pin = gpio.get(pin_num)?.into_output();
        
        let angle = Arc::new(Mutex::new(90)); // Start at center
        let running = Arc::new(Mutex::new(true));
        
        let angle_clone = Arc::clone(&angle);
        let running_clone = Arc::clone(&running);
        
        // Spawn background thread for continuous PWM generation
        // Uses high-priority timing to minimize jitter
        let _thread = thread::spawn(move || {
            let mut pin = pin;
            const PERIOD_US: u64 = 20_000; // 50Hz = 20ms period
            const PERIOD_DURATION: Duration = Duration::from_micros(PERIOD_US);
            
            // Cache angle to reduce lock contention
            let mut cached_angle = 90u8;
            let mut angle_update_counter = 0u32;
            
            loop {
                // Check if we should stop (non-blocking check)
                if !*running_clone.lock().unwrap() {
                    break;
                }
                
                // Update cached angle every 10 periods to reduce lock contention
                // This is safe because servo movement is slow (0.1s/60°)
                angle_update_counter += 1;
                if angle_update_counter >= 10 {
                    cached_angle = *angle_clone.lock().unwrap();
                    angle_update_counter = 0;
                }
                
                let angle = cached_angle.min(180) as u64;
                
                // Convert angle to pulse width in microseconds
                // 0° = 1000μs (1ms), 90° = 1500μs (1.5ms), 180° = 2000μs (2ms)
                // Linear interpolation: pulse = 1000 + (angle * 1000) / 180
                let pulse_width_us = 1000 + ((angle * 1000) / 180);
                
                // Generate one PWM period with precise timing
                let period_start = Instant::now();
                
                // High pulse - set pin high
                pin.set_high();
                let high_end = period_start + Duration::from_micros(pulse_width_us);
                
                // Precise busy-wait for high pulse duration
                // Don't use sleep or yield - they cause jitter
                busy_wait_until(high_end);
                
                // Low pulse - set pin low
                pin.set_low();
                let period_end = period_start + PERIOD_DURATION;
                
                // Precise busy-wait for remaining period time
                // This ensures exact 20ms period for stable 50Hz frequency
                busy_wait_until(period_end);
            }
        });
        
        Ok(SoftwarePwmServo {
            angle,
            running,
            _thread,
        })
    }

    fn set_angle(&self, angle: u8) -> Result<(), rppal::gpio::Error> {
        *self.angle.lock().unwrap() = angle.min(180);
        Ok(())
    }
}

impl Drop for SoftwarePwmServo {
    fn drop(&mut self) {
        *self.running.lock().unwrap() = false;
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Initializing MG90S servo test on GPIO 18 and 19...");
    println!("Both servos using software PWM with dedicated threads");

    // Initialize both servos using software PWM
    let servo1 = SoftwarePwmServo::new(18)
        .map_err(|e| format!("Failed to initialize servo on GPIO 18: {:?}", e))?;
    
    let servo2 = SoftwarePwmServo::new(19)
        .map_err(|e| format!("Failed to initialize servo on GPIO 19: {:?}", e))?;

    println!("Servos initialized. Starting test sequence...");
    
    // Small delay to let PWM threads stabilize
    thread::sleep(Duration::from_millis(100));

    // Test sequence: move both servos through different positions
    let positions = [(0, "0°"), (90, "90°"), (180, "180°"), (90, "90°")];

    for (angle, label) in positions.iter() {
        println!("Setting servos to {}...", label);
        
        // Set both servos to the same position
        servo1.set_angle(*angle)
            .map_err(|e| format!("Failed to set servo1 angle: {:?}", e))?;
        servo2.set_angle(*angle)
            .map_err(|e| format!("Failed to set servo2 angle: {:?}", e))?;
        
        // Wait for servo to reach position
        thread::sleep(Duration::from_secs(2));
    }

    println!("Test sequence completed. Servos holding at center position (90°).");
    
    // Keep servos at center position
    servo1.set_angle(90)
        .map_err(|e| format!("Failed to set servo1 to center: {:?}", e))?;
    servo2.set_angle(90)
        .map_err(|e| format!("Failed to set servo2 to center: {:?}", e))?;
    
    println!("Press Ctrl+C to exit...");
    
    // Keep the program running
    loop {
        thread::sleep(Duration::from_secs(1));
    }
}
