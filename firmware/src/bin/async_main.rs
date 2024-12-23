#![no_std]
#![no_main]
#![allow(warnings)]

use bt_hci::controller::ExternalController;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::i2c::master::Config;
use esp_hal::i2c::master::I2c;
use esp_hal::prelude::*;
use log::info;
use micromath::F32Ext;

use embedded_hal_async::i2c::I2c as _;

use esp_wifi::ble::controller::BleConnector;
use firmware::accelerometer::MotionDetector;
use firmware::ble_config::BleConfig;
use firmware::ble_config::BleConnectionState;
use firmware::ble_config::BleController;
use trouble_host::prelude::*;
extern crate alloc;

#[main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    esp_alloc::heap_allocator!(72 * 1024);

    esp_println::logger::init_logger_from_env();

    let timer0 = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER)
        .split::<esp_hal::timer::systimer::Target>();
    esp_hal_embassy::init(timer0.alarm0);

    info!("Embassy initialized!");

    let timer1 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    let init = esp_wifi::init(
        timer1.timer0,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    // let _ = spawner;
    // spawner.spawn(bluetooth_task(init, peripherals.BT)).unwrap();

    // .bluetooth();

    // TODO: Spawn some tasks

    let bluetooth = peripherals.BT;
    let connector = BleConnector::new(&init, bluetooth);
    let controller: ExternalController<_, 20> = ExternalController::new(connector);

    let mut config = BleConfig::default();
    config.device_name = "Eloy VBT";
    config.min_interval = Duration::from_millis(30);
    config.max_interval = Duration::from_millis(60);

    let mut ble = BleController::new(config);
    ble.subscribe_to_state_changes(|state| match state {
        BleConnectionState::Connected => info!("Device Connected!"),
        BleConnectionState::Disconnected => info!("Device Disconnected"),
        BleConnectionState::Advertising => info!("Advertising..."),
        BleConnectionState::Error(e) => info!("Error occurred: {:?}", e),
    })
    .unwrap();

    // ble.run(controller).await;
    // let mut i2c = I2c::new(peripherals.I2C0, Config::default())
    //     .with_sda(peripherals.GPIO2)
    //     .with_scl(peripherals.GPIO1)
    //     .into_async();

    let mut i2c = I2c::new(peripherals.I2C0, Config::default())
        .with_sda(peripherals.GPIO1)
        .with_scl(peripherals.GPIO2)
        .into_async();

    Timer::after(Duration::from_millis(100)).await;

    let mut motion_detector = match MotionDetector::new(i2c).await {
        Ok(detector) => detector,
        Err(e) => {
            // Handle initialization error
            info!("Err: {:?}", e);
            panic!("Failed to initialize motion detector");
        }
    };

    let mut up_velocity = 0.0f32;
    const DT: f32 = 0.1; // 100ms
                         //
    info!("Change here!");
    loop {
        // match motion_detector.wait_for_upward_movement().await {
        //     Ok(x) => info!("OK {x}"),
        //     Err(_) => info!("Falied"),
        // }
        // info!(
        //     "Test(Horizontal,Forward,Down) -->{:?}",
        //     motion_detector.get_accel_raw().await
        // );
        //
        //

        match motion_detector.get_next_motion_state().await {
            Ok(x) => {
                if x.is_up() {
                    // info!("UP --> {:?}", x.debug_str());
                    // let acc = x.get_acceleration();
                    // // up_velocity += acc.2 * DT;
                    // up_velocity += acc.2 * DT;
                    info!("UP velocity--> {:?}", x);
                    // info!("UP velocity: {} {:.2} m/s", acc.2, up_velocity);
                }

                if x.is_down() {
                    info!("DOWN --> {:?}", x);
                }
            }
            Err(e) => info!("Error --> {:?}", e),
        }
        Timer::after(Duration::from_millis(100)).await;
    }

    // let io = gpio::IO::new(peripherals.GPIO, peripherals.IO_MUX);
    // let sda = io.pins.gpio5;
    // let scl = io.pins.gpio6;
    // let i2c = I2c::new(peripherals.I2C0, sda, scl, 100u32.kHz(), &clocks);

    // Timer::after(Duration::from_millis(100)).await;
    // delay.delay_ms(100u32);

    // let mut accel = adxl345_eh_driver::Driver::new(i2c, None).unwrap();
    // let (x, y, z) = accel.get_accel_raw().unwrap();

    // // // info!("{x}{y}{z}");
    // let mut motion_detector = match MotionDetector::new(i2c).await {
    //     Ok(detector) => detector,
    //     Err(e) => {
    //         // Handle initialization error
    //         info!("Err: {:?}", e);
    //         panic!("Failed to initialize motion detector");
    //     }
    // };

    // loop {
    //     // Read motion periodically
    //     match motion_detector.read_motion().await {
    //         Ok(Some(motion)) => {
    //             // Motion detected
    //             info!("Vertical Velocity: {} m/s", motion.vertical_velocity);
    //         }
    //         Ok(None) => {
    //             // No significant motion
    //             // info!("No significant motion detected");
    //         }
    //         Err(_) => {
    //             // Handle I2C communication error
    //             info!("Error reading motion data");
    //         }
    //     }

    //     // Wait for a short duration before next reading
    //     Timer::after(Duration::from_millis(100)).await;
    // }

    // run_ble(controller).await;

    // loop {
    //     info!("Hello world!");
    //     Timer::after(Duration::from_secs(60)).await;
    // }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/v0.22.0/examples/src/bin
}
