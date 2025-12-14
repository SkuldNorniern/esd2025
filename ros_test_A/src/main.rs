// Simple ROS2 Publisher Example using rclrs
// Publishes std_msgs/String messages on /test_topic

use rclrs::*;
use std::time::Duration;

fn main() -> Result<(), RclrsError> {
    println!("ROS2 Publisher Test (ros_test_A)");
    println!("==================================");
    println!();

    // Check ROS environment variables that might block network communication
    println!("Checking ROS2 environment configuration...");
    let ros_domain_id = std::env::var("ROS_DOMAIN_ID")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    let ros_localhost_only = std::env::var("ROS_LOCALHOST_ONLY")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    let ros_automatic_discovery_range = std::env::var("ROS_AUTOMATIC_DISCOVERY_RANGE")
        .unwrap_or_else(|_| "SUBNET".to_string());

    println!("  ROS_DOMAIN_ID: {}", ros_domain_id);
    println!("  ROS_LOCALHOST_ONLY: {}", ros_localhost_only);
    println!("  ROS_AUTOMATIC_DISCOVERY_RANGE: {}", ros_automatic_discovery_range);
    
    if ros_localhost_only != 0 {
        eprintln!("  WARNING: ROS_LOCALHOST_ONLY is set! This will block network communication.");
        eprintln!("  Fix: unset ROS_LOCALHOST_ONLY or export ROS_LOCALHOST_ONLY=0");
    }
    if ros_automatic_discovery_range == "LOCALHOST" || ros_automatic_discovery_range == "OFF" {
        eprintln!("  WARNING: ROS_AUTOMATIC_DISCOVERY_RANGE is {}! This may block network communication.", ros_automatic_discovery_range);
        eprintln!("  Fix: export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET");
    }
    println!();

    // Initialize ROS2 context with domain ID from environment
    // Using default_from_env which reads ROS_DOMAIN_ID automatically
    let context = if ros_domain_id != 0 {
        let args = std::env::args();
        let init_options = InitOptions::new().with_domain_id(Some(ros_domain_id));
        Context::new(args, init_options)?
    } else {
        Context::default_from_env()?
    };

    // Create executor
    let mut executor = context.create_basic_executor();

    // Create node
    let node = executor.create_node("ros_test_publisher")?;

    println!("ROS2 node created: ros_test_publisher");
    println!("DDS Domain ID: {} (from ROS_DOMAIN_ID)", ros_domain_id);
    println!();

    // Create publisher for /test_topic
    // Using example_interfaces::msg::String as shown in rclrs docs
    // This is compatible with std_msgs/String in ROS 2
    println!("Creating ROS2 publisher on /test_topic...");
    let publisher = node.create_publisher::<example_interfaces::msg::String>("test_topic")?;

    println!("Publisher created successfully");
    println!();

    // Wait for publisher to establish connections
    // For cross-device communication, DDS discovery can take 10-30 seconds
    println!("Waiting for publisher to establish connections with subscribers...");
    println!("  (For cross-device: DDS discovery can take 10-30 seconds)");
    println!("  (For same-device: Usually 2-5 seconds)");
    for _i in 1..=15 {
        std::thread::sleep(Duration::from_secs(1));
        print!(".");
        std::io::Write::flush(&mut std::io::stdout()).ok();
    }
    println!();
    println!("Publisher ready, starting to publish messages...");
    println!("(Press Ctrl+C to stop)");
    println!();

    // Main publishing loop
    let mut message_count = 0u64;
    loop {
        message_count += 1;

        // Create message
        let msg = example_interfaces::msg::String {
            data: format!("Hello from ros_test_A! Message #{}", message_count),
        };

        // Publish message
        publisher.publish(&msg)?;

        if message_count % 10 == 0 {
            println!("Published message #{}", message_count);
        }

        // Publish at ~10 Hz (100ms delay)
        std::thread::sleep(Duration::from_millis(100));
    }
}
