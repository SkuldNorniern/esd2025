// ROS wrapper library providing channel-based interface for ROS2 topics
// Creates tokio channels (Sender/Receiver) that are backed by ROS publishers/subscribers

use r2r::{Context, Node};
use futures::stream::StreamExt;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::sync::mpsc;

// Re-export r2r so users can access all r2r types through ros_wrapper
pub use r2r;

// Re-export commonly used types for convenience
pub use r2r::QosProfile;
pub use r2r::std_msgs;
pub use r2r::sensor_msgs;

/// Error type for ROS wrapper operations
#[derive(Debug)]
pub enum RosWrapperError {
    /// ROS2 context creation failed
    ContextCreation(String),
    /// Node creation failed
    NodeCreation(String),
    /// Publisher creation failed
    PublisherCreation(String),
    /// Subscriber creation failed
    SubscriberCreation(String),
    /// Publishing failed
    Publish(String),
}

impl std::fmt::Display for RosWrapperError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RosWrapperError::ContextCreation(msg) => write!(f, "Context creation failed: {}", msg),
            RosWrapperError::NodeCreation(msg) => write!(f, "Node creation failed: {}", msg),
            RosWrapperError::PublisherCreation(msg) => write!(f, "Publisher creation failed: {}", msg),
            RosWrapperError::SubscriberCreation(msg) => write!(f, "Subscriber creation failed: {}", msg),
            RosWrapperError::Publish(msg) => write!(f, "Publish failed: {}", msg),
        }
    }
}

impl std::error::Error for RosWrapperError {}

/// Creates a Sender channel that publishes messages to a ROS topic
/// 
/// The returned Sender can be used to send messages that will be automatically
/// published to the specified ROS topic. A background task handles the publishing
/// and node spinning.
/// 
/// # Arguments
/// * `node_name` - Name of the ROS node
/// * `topic` - ROS topic name (e.g., "/test_topic")
/// * `qos` - Quality of Service profile (use QosProfile::default() for default)
/// 
/// # Returns
/// A tuple containing:
/// * `mpsc::Sender<T>` - Channel sender for publishing messages
/// * `Arc<Mutex<Node>>` - The ROS node wrapped in Arc<Mutex> (needs to be kept alive)
pub fn create_topic_sender<T>(
    node_name: &str,
    topic: &str,
    qos: r2r::QosProfile,
) -> Result<(mpsc::Sender<T>, Arc<Mutex<Node>>), RosWrapperError>
where
    T: r2r::WrappedTypesupport + Clone + Send + 'static,
{
    // Create ROS2 context
    let ctx = Context::create()
        .map_err(|e| RosWrapperError::ContextCreation(format!("{:?}", e)))?;
    
    // Create node
    let mut node = Node::create(ctx, node_name, "")
        .map_err(|e| RosWrapperError::NodeCreation(format!("{:?}", e)))?;
    
    // Create publisher
    let publisher = node.create_publisher::<T>(topic, qos)
        .map_err(|e| RosWrapperError::PublisherCreation(format!("{:?}", e)))?;
    
    // Create channel with larger capacity for high-throughput topics (e.g., images)
    // 500 frames at ~50KB each = ~25MB buffer
    let (tx, mut rx) = mpsc::channel::<T>(500);
    
    // Share node using Arc<Mutex> for background task
    let node_arc = Arc::new(Mutex::new(node));
    let node_for_spin = Arc::clone(&node_arc);
    
    // Spawn background task to handle publishing and node spinning
    tokio::spawn(async move {
        let spin_wait = Duration::from_millis(10); // Reduced from 50ms for faster processing
        let spin_once_timeout = Duration::from_millis(1); // Reduced for more responsive spinning
        let mut dropped_count = 0u64;
        let mut published_count = 0u64;

        loop {
            // Try to drain multiple messages quickly to catch up if falling behind
            let mut batch_count = 0;
            let max_batch = 10; // Process up to 10 messages per iteration
            
            loop {
                match rx.try_recv() {
                    Ok(msg) => {
                        // Publish message to ROS topic
                        if let Err(e) = publisher.publish(&msg) {
                            eprintln!("Failed to publish message: {:?}", e);
                        } else {
                            published_count += 1;
                        }
                        batch_count += 1;
                        
                        // Break after max_batch to allow spinning
                        if batch_count >= max_batch {
                            break;
                        }
                    }
                    Err(mpsc::error::TryRecvError::Empty) => {
                        // No more messages ready, break to spin
                        break;
                    }
                    Err(mpsc::error::TryRecvError::Disconnected) => {
                        // Sender dropped, exit task
                        return;
                    }
                }
            }
            
            // Spin the node to process DDS events
            if let Ok(mut node) = node_for_spin.lock() {
                node.spin_once(spin_once_timeout);
            }
            
            // If no messages were processed, wait a bit
            if batch_count == 0 {
                tokio::time::sleep(spin_wait).await;
            }
            
            // Log stats periodically
            if published_count > 0 && published_count % 100 == 0 {
                if dropped_count > 0 {
                    eprintln!("ROS publisher: {} published, {} dropped", published_count, dropped_count);
                }
            }
        }
    });
    
    Ok((tx, node_arc))
}

/// Creates a Receiver channel that receives messages from a ROS topic
/// 
/// The returned Receiver can be used to receive messages from the specified
/// ROS topic. A background task handles the subscription and node spinning.
/// 
/// # Arguments
/// * `node_name` - Name of the ROS node
/// * `topic` - ROS topic name (e.g., "/test_topic")
/// * `qos` - Quality of Service profile (use QosProfile::default() for default)
/// 
/// # Returns
/// A tuple containing:
/// * `mpsc::Receiver<T>` - Channel receiver for receiving messages
/// * `Arc<Mutex<Node>>` - The ROS node wrapped in Arc<Mutex> (needs to be kept alive)
pub fn create_topic_receiver<T>(
    node_name: &str,
    topic: &str,
    qos: r2r::QosProfile,
) -> Result<(mpsc::Receiver<T>, Arc<Mutex<Node>>), RosWrapperError>
where
    T: r2r::WrappedTypesupport + Send + 'static,
{
    // Create ROS2 context
    let ctx = Context::create()
        .map_err(|e| RosWrapperError::ContextCreation(format!("{:?}", e)))?;
    
    // Create node
    let mut node = Node::create(ctx, node_name, "")
        .map_err(|e| RosWrapperError::NodeCreation(format!("{:?}", e)))?;
    
    // Create subscriber
    let mut subscriber = node.subscribe::<T>(topic, qos)
        .map_err(|e| RosWrapperError::SubscriberCreation(format!("{:?}", e)))?;
    
    // Create channel with larger capacity for high-throughput topics (e.g., images)
    let (tx, rx) = mpsc::channel::<T>(500);
    
    // Share node using Arc<Mutex> for background task
    let node_arc = Arc::new(Mutex::new(node));
    let node_for_spin = Arc::clone(&node_arc);
    
    // Spawn background task to handle subscription and node spinning
    tokio::spawn(async move {
        let spin_wait = Duration::from_millis(50);
        let spin_once_timeout = Duration::from_millis(10);

        loop {
            match tokio::time::timeout(spin_wait, subscriber.next()).await {
                Ok(Some(msg)) => {
                    // Send message to channel
                    if tx.send(msg).await.is_err() {
                        // Receiver dropped, exit task
                        break;
                    }

                    // Also spin after forwarding so DDS events keep flowing under high receive rates.
                    if let Ok(mut node) = node_for_spin.lock() {
                        node.spin_once(Duration::from_millis(0));
                    }
                }
                Ok(None) => {
                    // Stream ended (e.g., context shut down)
                    break;
                }
                Err(_elapsed) => {
                    // Periodically spin the node to process DDS events
                    if let Ok(mut node) = node_for_spin.lock() {
                        node.spin_once(spin_once_timeout);
                    }
                }
            }
        }
    });
    
    Ok((rx, node_arc))
}


