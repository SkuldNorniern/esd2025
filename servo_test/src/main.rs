use software_pwm::SoftwarePwmServo;
use std::thread;
use std::time::Duration;

// MG90S servo specifications (from datasheet):
// - Operating frequency: 50Hz (20ms period)
// - Pulse width range: 1ms (-90°) to 2ms (+90°)
// - Neutral position: 1.5ms (0°)
// - Dead band width: 5μs
// - Operating speed: 0.1s/60° (4.8V), 0.08s/60° (6V)
// - Operating voltage: 4.8V-6.0V

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Initializing MG90S servo test on GPIO 18 and 19...");
    println!("Both servos using software PWM with dedicated threads");
    println!("Note: For best performance (minimal jitter), run with sudo or set CAP_SYS_NICE capability");

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
