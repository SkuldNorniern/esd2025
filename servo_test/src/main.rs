use software_pwm::create_servos;
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
    println!("Note: For best performance (minimal jitter), run with sudo or set CAP_SYS_NICE capability");

    // Initialize both servos using software PWM
    // Pin to CPU core 3 for best performance (adjust based on your system)
    let servos = create_servos(&[18, 19], Some(3))
        .map_err(|e| format!("Failed to initialize servos: {:?}", e))?;

    println!("Servos initialized. Starting test sequence...");
    thread::sleep(Duration::from_millis(100)); // Let PWM threads stabilize

    // Test sequence: move both servos through different positions
    for (angle, label) in [(0, "0°"), (90, "90°"), (180, "180°"), (90, "90°")] {
        println!("Setting servos to {}...", label);
        for servo in &servos {
            servo.set_angle(angle)?;
        }
        thread::sleep(Duration::from_secs(2));
    }

    println!("Test completed. Servos holding at center (90°).");
    for servo in &servos {
        servo.set_angle(90)?;
    }
    
    println!("Press Ctrl+C to exit...");
    loop {
        thread::sleep(Duration::from_secs(1));
    }
}
