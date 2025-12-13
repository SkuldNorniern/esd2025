// Simple ROS2 Subscriber Example
// Subscribes to std_msgs/String messages on /test_topic
// Based on ros2-client patterns from service examples

use ros2_client::{Context, NodeOptions, NodeName, MessageTypeName, Name};
use ros2_client::rustdds::QosPolicies;
use serde::{Deserialize, Serialize};
use std::time::{Duration, Instant};

// Message struct matching std_msgs/String format
#[derive(Serialize, Deserialize, Debug, Clone)]
struct StringMessage {
    data: String,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("ROS2 Subscriber Test (ros_test_B)");
    println!("===================================");
    println!();

    // Initialize ROS2 context
    let ctx = Context::new()
        .map_err(|e| format!("Failed to create ROS2 context: {:?}", e))?;

    // Create node
    let node_name = NodeName::new("/", "ros_test_subscriber")
        .map_err(|e| format!("Failed to create node name: {:?}", e))?;
    let mut node = ctx
        .new_node(node_name, NodeOptions::new().enable_rosout(true))
        .map_err(|e| format!("Failed to create ROS2 node: {:?}", e))?;

    // CRITICAL: Start the node spinner in a background thread to process DDS events
    // This is required for the subscription to receive messages!
    // Note: spinner.spin() returns a Future, but in synchronous code we run it in a thread
    // The spinner will process DDS events in the background
    let spinner = node.spinner()
        .map_err(|e| format!("Failed to create node spinner: {:?}", e))?;
    let _spinner_handle = std::thread::spawn(move || {
        // Run the spinner future in a blocking way
        // This processes DDS events continuously
        let _ = futures::executor::block_on(spinner.spin());
    });

    println!("ROS2 node created: ros_test_subscriber");
    println!("ROS_DOMAIN_ID: {}", std::env::var("ROS_DOMAIN_ID").unwrap_or_else(|_| "0".to_string()));
    println!();

    // Create subscriber for /test_topic
    println!("Subscribing to /test_topic...");
    let topic_name = Name::new("/", "test_topic")
        .map_err(|e| format!("Failed to create topic name: {:?}", e))?;
    let message_type = MessageTypeName::new("std_msgs", "String");

    let test_topic = node
        .create_topic(&topic_name, message_type, &QosPolicies::default())
        .map_err(|e| format!("Failed to create topic: {:?}", e))?;

    let subscriber = node
        .create_subscription::<StringMessage>(&test_topic, Some(QosPolicies::default()))
        .map_err(|e| format!("Failed to create subscriber: {:?}", e))?;

    println!("Subscriber created successfully");
    println!("  Topic: /test_topic");
    println!("  Message type: std_msgs/String");
    println!();

    // Wait for subscriber to establish connections
    println!("Waiting for subscriber to establish connections with publishers...");
    for _i in 1..=5 {
        std::thread::sleep(Duration::from_secs(1));
        print!(".");
        std::io::Write::flush(&mut std::io::stdout()).ok();
    }
    println!();
    println!("Subscriber ready, waiting for messages...");
    println!("(Press Ctrl+C to stop)");
    println!();

    // Event-driven message reception loop
    // Based on ros2-client service examples pattern
    let mut message_count = 0u64;
    let mut last_log_time = Instant::now();

    loop {
        // Event-driven polling: process all available messages in a batch
        loop {
            match subscriber.take() {
                Ok(Some((msg, _info))) => {
                    message_count += 1;

                    if message_count == 1 {
                        println!("✓ First message received!");
                    }

                    // Print received message
                    println!("Received message #{}: {}", message_count, msg.data);
                }
                Ok(None) => {
                    // No more messages available, break inner loop
                    break;
                }
                Err(e) => {
                    eprintln!("Error taking message from subscription: {:?}", e);
                    break;
                }
            }
        }

        // Log periodically if no messages received yet (rate limiting similar to service examples)
        if message_count == 0 {
            let now = Instant::now();
            if now.duration_since(last_log_time) > Duration::from_secs(2) {
                println!("Waiting for messages... (event loop polling)");
                last_log_time = now;
            }
        }

        // Small sleep to avoid busy-waiting (similar to service examples)
        std::thread::sleep(Duration::from_millis(10));
    }
}
