use rppal::gpio::Gpio;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

// MG90S servo specifications:
// - Operating frequency: 50Hz (20ms period)
// - Pulse width range: 1ms (0°) to 2ms (180°)
// - Neutral position: 1.5ms (90°)

// Software PWM servo with dedicated background thread
// Uses precise timing to minimize jitter
struct SoftwarePwmServo {
    angle: Arc<Mutex<u8>>,
    running: Arc<Mutex<bool>>,
    _thread: thread::JoinHandle<()>,
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
            let period_us = 20_000u64; // 50Hz = 20ms period
            
            loop {
                // Check if we should stop
                if !*running_clone.lock().unwrap() {
                    break;
                }
                
                // Get current angle
                let current_angle = *angle_clone.lock().unwrap();
                let angle = current_angle.min(180) as u64;
                
                // Convert angle to pulse width in microseconds
                // 0° = 1000μs, 180° = 2000μs
                let pulse_width_us = 1000 + ((angle * 1000) / 180);
                
                // Generate one PWM period with precise timing
                // Use Instant for accurate timing measurement
                let period_start = Instant::now();
                
                // High pulse
                pin.set_high();
                let high_start = Instant::now();
                
                // Busy-wait for high pulse duration
                // Use a combination of sleep and busy-wait for better accuracy
                if pulse_width_us > 1000 {
                    // For longer pulses, use sleep for most of the time
                    thread::sleep(Duration::from_micros(pulse_width_us - 500));
                }
                // Fine-tune with busy-wait for the remaining time
                while high_start.elapsed() < Duration::from_micros(pulse_width_us) {
                    // Small yield to prevent CPU spinning
                    thread::yield_now();
                }
                
                // Low pulse
                pin.set_low();
                let low_duration = period_us - pulse_width_us;
                
                // Wait for remaining period time
                if low_duration > 1000 {
                    thread::sleep(Duration::from_micros(low_duration - 500));
                }
                // Fine-tune to ensure exact 20ms period
                while period_start.elapsed() < Duration::from_micros(period_us) {
                    thread::yield_now();
                }
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
