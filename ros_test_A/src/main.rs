// Simple ROS2 Publisher Example using ros_wrapper
// Publishes std_msgs/String messages on /test_topic

use ros_wrapper::{create_topic_sender, QosProfile, std_msgs::msg::String as StringMsg};
use std::time::Duration;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
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

    // Create topic sender using ros_wrapper
    println!("Creating ROS2 publisher on /test_topic...");
    let (tx, _node) = create_topic_sender::<StringMsg>(
        "ros_test_publisher",
        "/test_topic",
        QosProfile::default(),
    )?;

    println!("Publisher created successfully");
    println!("DDS Domain ID: {} (from ROS_DOMAIN_ID)", ros_domain_id);
    println!();

    // Wait for publisher to establish connections
    // For cross-device communication, DDS discovery can take 10-30 seconds
    println!("Waiting for publisher to establish connections with subscribers...");
    println!("  (For cross-device: DDS discovery can take 10-30 seconds)");
    println!("  (For same-device: Usually 2-5 seconds)");
    for _i in 1..=15 {
        tokio::time::sleep(Duration::from_secs(1)).await;
        print!(".");
        std::io::Write::flush(&mut std::io::stdout())?;
    }
    println!();
    println!("Publisher ready, starting to publish messages...");
    println!("(Press Ctrl+C to stop)");
    println!();

    // Main publishing loop
    let mut message_count = 0u64;
    let mut interval = tokio::time::interval(Duration::from_millis(100));
    
    loop {
        interval.tick().await;
        message_count += 1;

        // Create message
        let msg = StringMsg {
            data: format!("Hello from ros_test_A! Message #{}", message_count),
        };

        // Send message through channel (will be published by background task)
        if let Err(e) = tx.send(msg).await {
            eprintln!("Failed to send message: {:?}", e);
            break;
        }

        if message_count % 10 == 0 {
            println!("Published message #{}", message_count);
        }
    }

    Ok(())
}
