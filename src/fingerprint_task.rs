use embassy_futures::select::{select, Either};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::mode::Async;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use crate::fingerprint_sensor::{
    ConfirmationCode, FingerError, GenImg, Img2Tz, LedColor, LedEffect, LedMode, RegModel, Search, Store,
    LIBRARY_CAPACITY,
};
use crate::request::{request, Reply};
use crate::unlock_task::UNLOCK_SIGNAL;
use crate::FingerprintSensor;

pub static FINGERPRINT_CHANNEL: Channel<CriticalSectionRawMutex, SensorCommand, 1> = Channel::new();

#[derive(Clone, Copy)]
pub enum SensorCommand {
    EnrollNewUser(Reply<()>),
}

static EFFECT_IN_PROGRESS: LedEffect = LedEffect {
    mode: LedMode::AlwaysOn,
    speed: 0x20,
    color: LedColor::Purple,
    cycles: 0x0,
};
static EFFECT_SUCCESS: LedEffect = LedEffect {
    mode: LedMode::Breathing,
    speed: 0x20,
    color: LedColor::Blue,
    cycles: 0x3,
};
static EFFECT_FAIL: LedEffect = LedEffect {
    mode: LedMode::Flashing,
    speed: 0x20,
    color: LedColor::Red,
    cycles: 0x3,
};

/// Scan attempts allowed per press before giving up.
const MAX_VALIDATE_ATTEMPTS: u8 = 5;
/// Pause between attempts: gives the user time to reposition and keeps a failed
/// scan from spinning the UART flat out on battery.
const VALIDATE_RETRY_DELAY: Duration = Duration::from_millis(300);
/// Interval between GenImg presence polls.
const PRESENCE_POLL: Duration = Duration::from_millis(100);

/// Owns the sensor and its WAKE pin.
///
/// WAKE is trusted only for the press, and only while the sensor is powered
/// down: powered, the sensor toggles it on its own (docs/wake-pin.md). While
/// powered, presence comes from GenImg instead, and every flow ends with the
/// finger confirmed off before power-down.
#[embassy_executor::task]
pub async fn fingerprint_manager_task(mut sensor: FingerprintSensor, mut wake: ExtiInput<'static, Async>) {
    loop {
        // Re-arm on release: WAKE can stay low with nothing GenImg can image
        // (wet glass, a flow that bailed on a bus error); a bare level wait
        // would then re-run validate back to back and never let the sensor sleep.
        let press = async {
            wake.wait_for_high().await;
            wake.wait_for_low().await
        };
        let cmd = select(press, FINGERPRINT_CHANNEL.receive()).await;
        sensor.enable().await.ok();

        match cmd {
            Either::First(()) => validate_access(&mut sensor).await,
            Either::Second(SensorCommand::EnrollNewUser(signal)) => {
                // Retry without signaling — the caller stays blocked until success.
                // A failed attempt needs a fresh placement, not the same one again.
                while enroll(&mut sensor).await.is_err() {
                    wait_lift(&mut sensor).await;
                }
                sensor.led_await(EFFECT_SUCCESS).await.ok();
                signal.signal(());
            }
        }

        sensor.disable();
    }
}

async fn validate_access(sensor: &mut FingerprintSensor) {
    // AlwaysOn with cycles: 0 — set once, stays lit across retries.
    sensor.run(EFFECT_IN_PROGRESS).await.ok();

    let effect = match scan_and_match(sensor).await {
        Scan::Match => {
            UNLOCK_SIGNAL.signal(());
            EFFECT_SUCCESS
        }
        Scan::NoMatch => EFFECT_FAIL,
        // Gone before a single image: too short a touch, or a stray WAKE low.
        Scan::NoFinger => {
            defmt::info!("wake without finger");
            return;
        }
    };
    show_until_lift(sensor, effect).await;
}

enum Scan {
    Match,
    NoMatch,
    NoFinger,
}

/// Scans until a match, the finger is lifted, or attempts run out.
async fn scan_and_match(sensor: &mut FingerprintSensor) -> Scan {
    for attempt in 1..=MAX_VALIDATE_ATTEMPTS {
        let result = async {
            sensor.run(GenImg).await?;
            sensor.run(Img2Tz(1)).await?;
            sensor.run(Search { buffer: 1, start: 0, count: LIBRARY_CAPACITY }).await
        }
        .await;

        match result {
            Ok(m) => {
                defmt::info!("fingerprint match: slot {} score {}", m.slot, m.score);
                return Scan::Match;
            }
            Err(FingerError::Sensor(ConfirmationCode::NoFinger)) if attempt == 1 => return Scan::NoFinger,
            Err(FingerError::Sensor(ConfirmationCode::NoFinger)) => return Scan::NoMatch,
            // Still on the sensor but unusable (NoMatch, ImageMessy,
            // FeatureFail) or a transient bus error — scan again.
            Err(_) if attempt < MAX_VALIDATE_ATTEMPTS => Timer::after(VALIDATE_RETRY_DELAY).await,
            Err(_) => {}
        }
    }
    Scan::NoMatch
}

/// Starts `effect` and returns once the finger is off and the animation has
/// played out, so powering down neither cuts it short nor leaves a finger on.
async fn show_until_lift(sensor: &mut FingerprintSensor, effect: LedEffect) {
    let end = Instant::now() + effect.duration();
    sensor.run(effect).await.ok();
    wait_lift(sensor).await;
    Timer::at(end).await;
}

/// Polls GenImg until a finger is on and its image captured.
async fn capture(sensor: &mut FingerprintSensor) -> Result<(), FingerError> {
    loop {
        match sensor.run(GenImg).await {
            // NoFinger, or on but the image failed — keep polling.
            Err(FingerError::Sensor(_)) => Timer::after(PRESENCE_POLL).await,
            other => return other,
        }
    }
}

/// Polls GenImg until the sensor reports no finger. Gives up on a bus error
/// rather than spinning on a dead sensor.
async fn wait_lift(sensor: &mut FingerprintSensor) {
    loop {
        match sensor.run(GenImg).await {
            Err(FingerError::Sensor(ConfirmationCode::NoFinger)) => return,
            Ok(()) | Err(FingerError::Sensor(_)) => Timer::after(PRESENCE_POLL).await,
            Err(_) => return,
        }
    }
}

async fn enroll(sensor: &mut FingerprintSensor) -> Result<(), FingerError> {
    // Pick the slot before scanning so a full library fails
    // without making the user press twice for nothing.
    let slot = sensor.find_free_slot().await?;
    for i in 1..=2u8 {
        sensor.run(EFFECT_IN_PROGRESS).await?;
        capture(sensor).await?;
        sensor.run(Img2Tz(i)).await?;
        sensor.run(EFFECT_SUCCESS).await?;
        wait_lift(sensor).await;
    }
    sensor.run(RegModel).await?;
    sensor.run(Store { buffer: 1, slot }).await
}

#[embassy_executor::task]
pub async fn add_new_finger_task(mut pin: ExtiInput<'static, Async>) {
    static DONE: Signal<CriticalSectionRawMutex, ()> = Signal::new();
    loop {
        pin.wait_for_low().await;

        request(&FINGERPRINT_CHANNEL, &DONE, SensorCommand::EnrollNewUser).await;
    }
}
