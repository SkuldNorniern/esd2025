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
    
    // Create channel with moderate capacity for image topics
    // 200 frames at ~50KB each = ~10MB buffer (balanced approach)
    let (tx, mut rx) = mpsc::channel::<T>(200);
    
    // Share node using Arc<Mutex> for background task
    let node_arc = Arc::new(Mutex::new(node));
    let node_for_spin = Arc::clone(&node_arc);
    
    // Spawn background task to handle publishing and node spinning
    tokio::spawn(async move {
        loop {
            // Try to get a message without blocking (use try_recv for high throughput)
            match rx.try_recv() {
                Ok(msg) => {
                    // Publish message to ROS topic
                    if let Err(e) = publisher.publish(&msg) {
                        eprintln!("Failed to publish message: {:?}", e);
                    }
                    
                    // Spin briefly after each publish to process DDS events
                    // Use try_lock to avoid blocking if someone else has the lock
                    if let Ok(mut node) = node_for_spin.try_lock() {
                        // Use zero timeout to avoid blocking while holding the lock
                        node.spin_once(Duration::from_millis(0));
                    }
                    
                    // Yield to allow other tasks to run (critical for tokio fairness)
                    tokio::task::yield_now().await;
                }
                Err(mpsc::error::TryRecvError::Empty) => {
                    // No messages available, spin the node and wait briefly
                    // Use try_lock to avoid blocking if someone else has the lock
                    if let Ok(mut node) = node_for_spin.try_lock() {
                        // Use zero timeout to avoid blocking while holding the lock
                        node.spin_once(Duration::from_millis(0));
                    }
                    
                    // Small sleep to avoid busy-waiting
                    tokio::time::sleep(Duration::from_micros(500)).await;
                }
                Err(mpsc::error::TryRecvError::Disconnected) => {
                    // Sender dropped, exit task
                    break;
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
    
    // Create channel with moderate capacity for image topics
    let (tx, rx) = mpsc::channel::<T>(200);
    
    // Share node using Arc<Mutex> for background task
    let node_arc = Arc::new(Mutex::new(node));
    let node_for_spin = Arc::clone(&node_arc);
    
    // Spawn background task to handle subscription and node spinning
    tokio::spawn(async move {
        let spin_wait = Duration::from_millis(50);

        loop {
            match tokio::time::timeout(spin_wait, subscriber.next()).await {
                Ok(Some(msg)) => {
                    // Send message to channel
                    if tx.send(msg).await.is_err() {
                        // Receiver dropped, exit task
                        break;
                    }

                    // Also spin after forwarding so DDS events keep flowing under high receive rates.
                    // Use try_lock to avoid blocking if someone else has the lock
                    if let Ok(mut node) = node_for_spin.try_lock() {
                        // Use zero timeout to avoid blocking while holding the lock
                        node.spin_once(Duration::from_millis(0));
                    }
                }
                Ok(None) => {
                    // Stream ended (e.g., context shut down)
                    break;
                }
                Err(_elapsed) => {
                    // Periodically spin the node to process DDS events
                    // Use try_lock to avoid blocking if someone else has the lock
                    if let Ok(mut node) = node_for_spin.try_lock() {
                        // Use zero timeout to avoid blocking while holding the lock
                        node.spin_once(Duration::from_millis(0));
                    }
                }
            }
        }
    });
    
    Ok((rx, node_arc))
}


