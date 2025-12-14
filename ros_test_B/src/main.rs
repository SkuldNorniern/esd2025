// Simple ROS2 Subscriber Example using rclrs
// Subscribes to std_msgs/String messages on /test_topic

use rclrs::*;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

fn main() -> Result<(), RclrsError> {
    println!("ROS2 Subscriber Test (ros_test_B)");
    println!("===================================");
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
    let node = executor.create_node("ros_test_subscriber")?;

    println!("ROS2 node created: ros_test_subscriber");
    println!("DDS Domain ID: {} (from ROS_DOMAIN_ID)", ros_domain_id);
    println!();

    // Create subscriber for /test_topic
    println!("Subscribing to /test_topic...");
    
    // Use atomic counter to track message count from callback
    let message_count = Arc::new(AtomicU64::new(0));
    let message_count_clone = Arc::clone(&message_count);
    let first_message_received = Arc::new(std::sync::atomic::AtomicBool::new(false));
    let first_message_received_clone = Arc::clone(&first_message_received);

    // Using example_interfaces::msg::String as shown in rclrs docs
    // This is compatible with std_msgs/String in ROS 2
    let _subscription = node.create_subscription(
        "test_topic",
        move |msg: example_interfaces::msg::String| {
            let count = message_count_clone.fetch_add(1, Ordering::Relaxed) + 1;
            
            if !first_message_received_clone.swap(true, Ordering::Relaxed) {
                println!("First message received!");
            }
            
            println!("Received message #{}: {}", count, msg.data);
        },
    )?;

    println!("Subscriber created successfully");
    println!("  Topic: /test_topic");
    println!("  Message type: std_msgs/String (using example_interfaces::msg::String)");
    println!();

    // Wait for subscriber to establish connections
    // For cross-device communication, DDS discovery can take 10-30 seconds
    println!("Waiting for subscriber to establish connections with publishers...");
    println!("  (For cross-device: DDS discovery can take 10-30 seconds)");
    println!("  (For same-device: Usually 2-5 seconds)");
    println!("  Check connection status with: ros2 topic info /test_topic");
    for _i in 1..=15 {
        std::thread::sleep(Duration::from_secs(1));
        print!(".");
        std::io::Write::flush(&mut std::io::stdout()).ok();
    }
    println!();
    println!("Subscriber ready, waiting for messages...");
    println!("(Press Ctrl+C to stop)");
    println!();
    println!("Diagnostics:");
    println!("  - Run 'ros2 topic info /test_topic' to check publisher/subscriber count");
    println!("  - Run 'ros2 topic echo /test_topic' to verify messages are published");
    println!("  - If messages still not received, try increasing wait time or check firewall");
    println!();

    // Start a thread to periodically log if no messages received yet
    let message_count_log = Arc::clone(&message_count);
    let _log_thread = std::thread::spawn(move || {
        let mut last_log_time = Instant::now();
        loop {
            std::thread::sleep(Duration::from_secs(2));
            if message_count_log.load(Ordering::Relaxed) == 0 {
                let now = Instant::now();
                if now.duration_since(last_log_time) > Duration::from_secs(2) {
                    println!("Waiting for messages...");
                    println!("  Troubleshooting:");
                    println!("    - Verify publisher is running: ros2 node list");
                    println!("    - Check topic info: ros2 topic info /test_topic");
                    println!("    - Verify messages: ros2 topic echo /test_topic");
                    println!("    - Ensure ROS_DOMAIN_ID matches on both devices");
                    println!("    - Check firewall: UDP ports 7400-7500 should be open");
                    last_log_time = now;
                }
            }
        }
    });

    // Spin the executor to process incoming messages
    executor.spin(SpinOptions::default()).first_error()?;

    Ok(())
}
