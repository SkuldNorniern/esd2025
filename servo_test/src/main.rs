use rppal::gpio::{Gpio, OutputPin};
use std::sync::{
    atomic::{AtomicBool, AtomicU16, Ordering},
    Arc,
};
use std::thread;

const PERIOD_US: u64 = 20_000; // 50Hz

// MG90S typical: ~1000..2000us (sometimes needs 500..2500 depending on unit)
// Keep conservative unless you know your endpoints.
const MIN_US: u16 = 1000;
const MAX_US: u16 = 2000;

fn angle_to_pulse_us(angle: u16) -> u16 {
    let a = angle.min(180);
    MIN_US + ((a * (MAX_US - MIN_US)) / 180)
}

// --------- low-level monotonic time helpers (absolute scheduling) ---------

fn now_mono_ns() -> u64 {
    unsafe {
        let mut ts: libc::timespec = std::mem::zeroed();
        libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts);
        (ts.tv_sec as u64) * 1_000_000_000u64 + (ts.tv_nsec as u64)
    }
}

fn sleep_until_mono_ns(target_ns: u64) {
    unsafe {
        let ts = libc::timespec {
            tv_sec: (target_ns / 1_000_000_000u64) as libc::time_t,
            tv_nsec: (target_ns % 1_000_000_000u64) as libc::c_long,
        };
        // Absolute sleep on CLOCK_MONOTONIC
        libc::clock_nanosleep(libc::CLOCK_MONOTONIC, libc::TIMER_ABSTIME, &ts, std::ptr::null_mut());
    }
}

fn spin_until_mono_ns(target_ns: u64) {
    while now_mono_ns() < target_ns {
        std::hint::spin_loop();
    }
}

// Try to make the PWM thread “win” against Linux.
// If this fails, it still runs; you just get more jitter.
fn try_make_realtime_and_pin() {
    unsafe {
        // SCHED_FIFO priority: 1..99
        let param = libc::sched_param { sched_priority: 80 };
        libc::sched_setscheduler(0, libc::SCHED_FIFO, &param);

        // Pin to CPU 0 (helps avoid migration jitter)
        let mut set: libc::cpu_set_t = std::mem::zeroed();
        libc::CPU_ZERO(&mut set);
        libc::CPU_SET(0, &mut set);
        libc::sched_setaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &set);
    }
}

pub struct DualServoPwm {
    pulse1_us: Arc<AtomicU16>,
    pulse2_us: Arc<AtomicU16>,
    running: Arc<AtomicBool>,
    _thread: thread::JoinHandle<()>,
}

impl DualServoPwm {
    pub fn new(pin1: u8, pin2: u8) -> Result<Self, Box<dyn std::error::Error>> {
        let gpio = Gpio::new()?;
        let p1 = gpio.get(pin1)?.into_output_low();
        let p2 = gpio.get(pin2)?.into_output_low();

        let pulse1_us = Arc::new(AtomicU16::new(angle_to_pulse_us(90)));
        let pulse2_us = Arc::new(AtomicU16::new(angle_to_pulse_us(90)));
        let running = Arc::new(AtomicBool::new(true));

        let pulse1_c = pulse1_us.clone();
        let pulse2_c = pulse2_us.clone();
        let running_c = running.clone();

        let _thread = thread::spawn(move || pwm_loop(p1, p2, pulse1_c, pulse2_c, running_c));

        Ok(Self {
            pulse1_us,
            pulse2_us,
            running,
            _thread,
        })
    }

    pub fn set_angle_1(&self, angle: u16) {
        self.pulse1_us.store(angle_to_pulse_us(angle), Ordering::Relaxed);
    }

    pub fn set_angle_2(&self, angle: u16) {
        self.pulse2_us.store(angle_to_pulse_us(angle), Ordering::Relaxed);
    }

    pub fn set_angles(&self, a1: u16, a2: u16) {
        self.set_angle_1(a1);
        self.set_angle_2(a2);
    }
}

impl Drop for DualServoPwm {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Relaxed);
        // Optionally join, but don't block Drop if you don't want to.
        // let _ = self._thread.join();
    }
}

fn pwm_loop(
    mut s1: OutputPin,
    mut s2: OutputPin,
    pulse1_us: Arc<AtomicU16>,
    pulse2_us: Arc<AtomicU16>,
    running: Arc<AtomicBool>,
) {
    try_make_realtime_and_pin();

    let period_ns = PERIOD_US * 1_000;
    let mut next_start = now_mono_ns() + period_ns;

    // For best edges: sleep most of the time, spin only near deadlines.
    // Tune this (100..500us) if needed.
    let spin_guard_ns: u64 = 200_000; // 200us

    while running.load(Ordering::Relaxed) {
        // Absolute wait for start of frame
        let now = now_mono_ns();
        if next_start > now + spin_guard_ns {
            sleep_until_mono_ns(next_start - spin_guard_ns);
        }
        spin_until_mono_ns(next_start);

        let start = next_start;

        let p1 = pulse1_us.load(Ordering::Relaxed).clamp(MIN_US, MAX_US) as u64;
        let p2 = pulse2_us.load(Ordering::Relaxed).clamp(MIN_US, MAX_US) as u64;

        // Raise both at the same time
        s1.set_high();
        s2.set_high();

        // Decide which falls first
        let (first_is_1, t_first_ns, t_second_ns) = if p1 <= p2 {
            (true, start + p1 * 1_000, start + p2 * 1_000)
        } else {
            (false, start + p2 * 1_000, start + p1 * 1_000)
        };

        // First falling edge
        if t_first_ns > now_mono_ns() + spin_guard_ns {
            sleep_until_mono_ns(t_first_ns - spin_guard_ns);
        }
        spin_until_mono_ns(t_first_ns);
        if first_is_1 {
            s1.set_low();
        } else {
            s2.set_low();
        }

        // Second falling edge
        if t_second_ns > now_mono_ns() + spin_guard_ns {
            sleep_until_mono_ns(t_second_ns - spin_guard_ns);
        }
        spin_until_mono_ns(t_second_ns);
        if first_is_1 {
            s2.set_low();
        } else {
            s1.set_low();
        }

        // End of period
        next_start = start + period_ns;
        // Ensure both are low (safety)
        s1.set_low();
        s2.set_low();
    }

    // Clean shutdown
    s1.set_low();
    s2.set_low();
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Dual MG90S software PWM test (single RT thread).");

    // Use your actual pins here (you said 17 and 27 in your build)
    let servos = DualServoPwm::new(17, 27)?;

    // Simple sweep test
    loop {
        servos.set_angles(0, 0);
        std::thread::sleep(std::time::Duration::from_secs(2));

        servos.set_angles(90, 90);
        std::thread::sleep(std::time::Duration::from_secs(2));

        servos.set_angles(180, 180);
        std::thread::sleep(std::time::Duration::from_secs(2));

        servos.set_angles(90, 90);
        std::thread::sleep(std::time::Duration::from_secs(2));
    }
}
