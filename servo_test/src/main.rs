use software_pwm::SoftwarePwmServo;
use std::thread;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("MG90S Servo Test");
    println!("GPIO 18 and 19 using software PWM");
    println!("Note: Run with sudo for best performance (real-time scheduling)");

    // Initialize servos (pin to CPU core 3 for best performance)
    let servo1 = SoftwarePwmServo::new(18, Some(3))?;
    let servo2 = SoftwarePwmServo::new(19, Some(3))?;

    println!("Servos initialized. Starting test sequence...");
    thread::sleep(Duration::from_millis(100));

    // Test sequence
    for (angle, label) in [(0, "0°"), (90, "90°"), (180, "180°"), (90, "90°")] {
        println!("Setting servos to {}...", label);
        servo1.set_angle(angle)?;
        servo2.set_angle(angle)?;
        thread::sleep(Duration::from_secs(2));
    }

    println!("Test completed. Servos at center (90°). Press Ctrl+C to exit.");
    loop {
        thread::sleep(Duration::from_secs(1));
    }
}
