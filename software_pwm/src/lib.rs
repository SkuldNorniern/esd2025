// Software PWM library for precise servo control
// Uses atomics, real-time scheduling, and CPU affinity for minimal jitter

use rppal::gpio::Gpio;
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

// Precise busy-wait function optimized for minimal jitter
// Uses CPU cycles for accurate microsecond-level timing
// Inlined and optimized to reduce function call overhead
#[inline(always)]
fn busy_wait_until(target: Instant) {
    // Tight loop with memory barriers to prevent reordering
    // Use spin_loop hint for better CPU pipeline utilization
    while Instant::now() < target {
        std::hint::spin_loop();
        // Compiler barrier to prevent loop optimization
        std::sync::atomic::compiler_fence(Ordering::Acquire);
    }
}

// Set thread to real-time priority and pin to specific CPU core
// This reduces jitter from thread scheduling and context switches
fn optimize_thread_for_realtime(cpu_core: Option<usize>) -> Result<(), String> {
    // Set real-time scheduling policy (requires CAP_SYS_NICE or root)
    #[cfg(target_os = "linux")]
    {
        use thread_priority::{ThreadPriority, ThreadSchedulePolicy, RealtimeThreadSchedulePolicy, ThreadPriorityValue};
        use thread_priority::unix::{set_thread_priority_and_policy, thread_native_id};
        
        // Get current thread ID
        let thread_id = thread_native_id();
        
        // Try to set real-time priority with SCHED_FIFO policy
        // SCHED_FIFO gives highest priority, preempts normal threads
        let policy = ThreadSchedulePolicy::Realtime(RealtimeThreadSchedulePolicy::Fifo);
        
        // Use priority 50 (reasonable value, 1-99 range for SCHED_FIFO)
        // try_from returns Result, not Option
        match ThreadPriorityValue::try_from(50) {
            Ok(priority_value) => {
                if let Err(e) = set_thread_priority_and_policy(
                    thread_id,
                    ThreadPriority::Crossplatform(priority_value),
                    policy
                ) {
                    // If real-time fails, try to at least increase priority
                    if let Ok(fallback_priority) = ThreadPriorityValue::try_from(10) {
                        use thread_priority::unix::set_thread_priority;
                        if let Err(_) = set_thread_priority(thread_id, ThreadPriority::Crossplatform(fallback_priority)) {
                            return Err(format!("Failed to set thread priority: {:?}. Run with sudo or set CAP_SYS_NICE capability", e));
                        }
                    }
                }
            }
            Err(_) => {
                // Try lower priority if 50 fails
                if let Ok(priority_value) = ThreadPriorityValue::try_from(1) {
                    if let Err(e) = set_thread_priority_and_policy(
                        thread_id,
                        ThreadPriority::Crossplatform(priority_value),
                        policy
                    ) {
                        return Err(format!("Failed to set thread priority: {:?}. Run with sudo or set CAP_SYS_NICE capability", e));
                    }
                }
            }
        }
        
        // Set CPU affinity to pin thread to specific core
        if let Some(core) = cpu_core {
            use libc::{cpu_set_t, CPU_SET, CPU_ZERO, sched_setaffinity};
            use std::mem::MaybeUninit;
            
            let mut cpuset: cpu_set_t = unsafe { MaybeUninit::zeroed().assume_init() };
            unsafe {
                CPU_ZERO(&mut cpuset);
                CPU_SET(core, &mut cpuset);
                
                // Set CPU affinity for current thread (pid 0 = current thread)
                if sched_setaffinity(0, std::mem::size_of::<cpu_set_t>(), &cpuset) != 0 {
                    return Err(format!("Failed to set CPU affinity to core {}. Error: {}", core, std::io::Error::last_os_error()));
                }
            }
        }
    }
    
    #[cfg(not(target_os = "linux"))]
    {
        let _ = cpu_core; // Suppress unused warning
        return Err("Real-time optimizations only available on Linux".to_string());
    }
    
    Ok(())
}

// Software PWM servo with dedicated background thread
// Uses precise timing to minimize jitter
// Uses atomics to eliminate lock contention
pub struct SoftwarePwmServo {
    angle: Arc<AtomicU8>,
    running: Arc<AtomicBool>,
    _thread: thread::JoinHandle<()>,
}

impl SoftwarePwmServo {
    /// Create a new software PWM servo controller
    /// 
    /// # Arguments
    /// * `pin_num` - GPIO pin number (BCM numbering on Raspberry Pi)
    /// * `cpu_core` - Optional CPU core to pin the PWM thread to (None = no affinity)
    /// 
    /// # Returns
    /// * `Ok(SoftwarePwmServo)` on success
    /// * `Err(rppal::gpio::Error)` on failure
    /// 
    /// # Example
    /// ```
    /// let servo = SoftwarePwmServo::new(18, Some(3))?; // Pin to CPU core 3
    /// servo.set_angle(90)?; // Set to center position
    /// ```
    pub fn new(pin_num: u8, cpu_core: Option<usize>) -> Result<Self, rppal::gpio::Error> {
        let gpio = Gpio::new()?;
        let pin = gpio.get(pin_num)?.into_output();
        
        let angle = Arc::new(AtomicU8::new(90)); // Start at center
        let running = Arc::new(AtomicBool::new(true));
        
        let angle_clone = Arc::clone(&angle);
        let running_clone = Arc::clone(&running);
        
        // Spawn background thread for continuous PWM generation
        // Uses high-priority timing, real-time scheduling, and CPU affinity to minimize jitter
        let cpu_core_for_thread = cpu_core;
        let _thread = thread::spawn(move || {
            // Optimize thread for real-time performance
            // This must be done in the thread itself, not before spawning
            if let Err(e) = optimize_thread_for_realtime(cpu_core_for_thread) {
                eprintln!("Warning: Failed to optimize PWM thread for real-time: {}", e);
                eprintln!("  PWM will still work but may have more jitter");
                eprintln!("  For best performance, run with sudo or set CAP_SYS_NICE capability");
            }
            
            let mut pin = pin;
            const PERIOD_US: u64 = 20_000; // 50Hz = 20ms period
            const PERIOD_DURATION: Duration = Duration::from_micros(PERIOD_US);
            
            // Pre-calculate common values to reduce computation in hot loop
            const ANGLE_UPDATE_INTERVAL: u32 = 10;
            const MIN_ANGLE: u8 = 0;
            const MAX_ANGLE: u8 = 180;
            const PULSE_MIN_US: u64 = 1000; // 0° = 1ms
            const PULSE_MAX_US: u64 = 2000; // 180° = 2ms
            const PULSE_RANGE_US: u64 = PULSE_MAX_US - PULSE_MIN_US; // 1000μs
            
            // Cache angle to reduce atomic reads
            let mut cached_angle = 90u8;
            let mut angle_update_counter = 0u32;
            
            // Pre-calculate pulse width for cached angle to reduce computation
            let mut cached_pulse_width_us = 1500u64; // Default to center (90°)
            
            loop {
                // Check if we should stop (non-blocking atomic check - no locks!)
                // Use Acquire ordering to ensure we see the latest value
                if !running_clone.load(Ordering::Acquire) {
                    break;
                }
                
                // Read angle atomically - no lock needed, always fast
                // Update cached angle every N periods to reduce atomic reads
                // This is safe because servo movement is slow (0.1s/60°)
                angle_update_counter += 1;
                if angle_update_counter >= ANGLE_UPDATE_INTERVAL {
                    let new_angle = angle_clone.load(Ordering::Acquire);
                    if new_angle != cached_angle {
                        cached_angle = new_angle.min(MAX_ANGLE);
                        // Pre-calculate pulse width: 1000 + (angle * 1000) / 180
                        // Using optimized integer math
                        let angle_u64 = cached_angle as u64;
                        cached_pulse_width_us = PULSE_MIN_US + ((angle_u64 * PULSE_RANGE_US) / MAX_ANGLE as u64);
                    }
                    angle_update_counter = 0;
                }
                
                // Generate one PWM period with precise timing
                // Capture start time immediately before setting pin
                // Use memory barrier to ensure pin operation happens after time capture
                std::sync::atomic::compiler_fence(Ordering::SeqCst);
                let period_start = Instant::now();
                
                // High pulse - set pin high
                // Pin operations are fast but we want minimal delay
                pin.set_high();
                
                // Calculate target time for high pulse end
                // Pre-calculated pulse width reduces computation here
                let high_end = period_start + Duration::from_micros(cached_pulse_width_us);
                
                // Precise busy-wait for high pulse duration
                // Don't use sleep or yield - they cause jitter
                busy_wait_until(high_end);
                
                // Low pulse - set pin low
                pin.set_low();
                
                // Calculate target time for period end (exactly 20ms from start)
                let period_end = period_start + PERIOD_DURATION;
                
                // Precise busy-wait for remaining period time
                // This ensures exact 20ms period for stable 50Hz frequency
                // Critical for servo stability - any jitter here causes servo jitter
                busy_wait_until(period_end);
            }
        });
        
        Ok(SoftwarePwmServo {
            angle,
            running,
            _thread,
        })
    }

    /// Set the servo angle
    /// 
    /// # Arguments
    /// * `angle` - Angle in degrees (0-180, clamped automatically)
    /// 
    /// # Returns
    /// * `Ok(())` on success
    /// * `Err(rppal::gpio::Error)` on failure (unlikely for this operation)
    /// 
    /// # Example
    /// ```
    /// servo.set_angle(90)?; // Center position
    /// servo.set_angle(0)?;  // Leftmost position
    /// servo.set_angle(180)?; // Rightmost position
    /// ```
    pub fn set_angle(&self, angle: u8) -> Result<(), rppal::gpio::Error> {
        // Atomic write - no lock needed, always fast
        self.angle.store(angle.min(180), Ordering::Relaxed);
        Ok(())
    }
}

impl Drop for SoftwarePwmServo {
    fn drop(&mut self) {
        // Atomic write - no lock needed
        self.running.store(false, Ordering::Relaxed);
    }
}
