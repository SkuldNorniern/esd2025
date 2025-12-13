// Simple ROS2 Publisher Example
// Publishes std_msgs/String messages on /test_topic
// Based on ros2-client patterns from service examples

use ros2_client::{Context, NodeName, NodeOptions, MessageTypeName, Name};
use ros2_client::rustdds::QosPolicies;
use serde::Serialize;
use std::time::Duration;

// Message struct matching std_msgs/String format
#[derive(Serialize, Debug, Clone)]
struct StringMessage {
    data: String,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("ROS2 Publisher Test (ros_test_A)");
    println!("==================================");
    println!();

    // Initialize ROS2 context
    let ctx = Context::new()
        .map_err(|e| format!("Failed to create ROS2 context: {:?}", e))?;

    // Create node
    let node_name = NodeName::new("/", "ros_test_publisher")
        .map_err(|e| format!("Failed to create node name: {:?}", e))?;
    let mut node = ctx
        .new_node(node_name, NodeOptions::new().enable_rosout(true))
        .map_err(|e| format!("Failed to create ROS2 node: {:?}", e))?;

    // CRITICAL: Start the node spinner in a background thread to process DDS events
    // This is required for the publisher to send messages properly!
    // Note: spinner.spin() returns a Future, but in synchronous code we run it in a thread
    // The spinner will process DDS events in the background
    let spinner = node.spinner()
        .map_err(|e| format!("Failed to create node spinner: {:?}", e))?;
    let _spinner_handle = std::thread::spawn(move || {
        // Run the spinner future in a blocking way
        // This processes DDS events continuously
        let _ = futures::executor::block_on(spinner.spin());
    });

    println!("ROS2 node created: ros_test_publisher");
    println!("ROS_DOMAIN_ID: {}", std::env::var("ROS_DOMAIN_ID").unwrap_or_else(|_| "0".to_string()));
    println!();

    // Create publisher for /test_topic
    println!("Creating ROS2 publisher on /test_topic...");
    let topic_name = Name::new("/", "test_topic")
        .map_err(|e| format!("Failed to create topic name: {:?}", e))?;
    let message_type = MessageTypeName::new("std_msgs", "String");

    let test_topic = node
        .create_topic(&topic_name, message_type, &QosPolicies::default())
        .map_err(|e| format!("Failed to create topic: {:?}", e))?;

    let publisher = node
        .create_publisher(&test_topic, Some(QosPolicies::default()))
        .map_err(|e| format!("Failed to create publisher: {:?}", e))?;

    println!("Publisher created successfully");
    println!();

    // Wait for publisher to establish connections
    println!("Waiting for publisher to establish connections...");
    for _i in 1..=5 {
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
        let msg = StringMessage {
            data: format!("Hello from ros_test_A! Message #{}", message_count),
        };

        // Publish message
        match publisher.publish(msg) {
            Ok(()) => {
                if message_count % 10 == 0 {
                    println!("Published message #{}", message_count);
                }
            }
            Err(e) => {
                let error_str = format!("{:?}", e);
                if error_str.contains("WouldBlock") {
                    // WouldBlock is transient, just continue
                    if message_count % 100 == 0 {
                        eprintln!("Warning: WouldBlock on message #{} (publisher buffer may be full)", message_count);
                    }
                } else {
                    eprintln!("ERROR: Failed to publish message #{}: {:?}", message_count, e);
                    return Err(format!("Failed to publish: {:?}", e).into());
                }
            }
        }

        // Publish at ~10 Hz (100ms delay)
        std::thread::sleep(Duration::from_millis(100));
    }
}
