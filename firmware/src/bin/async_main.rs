#![no_std]
#![no_main]
#![allow(warnings)]

use bt_hci::controller::ExternalController;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::prelude::*;
use esp_wifi::ble::*;
use log::info;

use esp_hal::{rng::Rng, time, timer::timg::TimerGroup};
use esp_wifi::{ble::controller::BleConnector, init};

use trouble_host::prelude::*;

extern crate alloc;

use alloc::format;

// GATT Server definition
#[gatt_server]
struct Server {
    random_service: RandomService,
}

// Random value service
#[gatt_service(uuid = "1234")] // Use your own custom UUID
struct RandomService {
    #[characteristic(uuid = "2A58", read, notify)]
    random_value: u8,
}

async fn ble_task<C: Controller>(
    server: &Server<'_>,
    connection: Connection<'_>,
    stack: Stack<'_, C>,
) {
    info!(
        "BLE Task Started - Connection Handle: {:?}",
        connection.handle()
    );
    let mut i = 0;
    // Add more robust event handling
    loop {
        let random_value = server.random_service.random_value;
        i += 1;
        if (i > 50) {
            i = 0;
        }
        let oo = server.set(&random_value, &i);
        match connection.next().await {
            ConnectionEvent::Gatt { data } => {
                info!("GATT event: {:?}", data.request());

                // Process the service discovery request
                match data.process(server).await {
                    Ok(Some(event)) => {
                        match event {
                            GattEvent::Read(read_event) => {
                                info!("Read event for handle: {}", read_event.handle());
                                // Optionally respond or log
                                let _ = read_event.reply(Ok(())).await;
                            }
                            GattEvent::Write(write_event) => {
                                info!("Write event for handle: {}", write_event.handle());
                                let _ = write_event.reply(Ok(())).await;
                            }
                        }
                    }
                    Ok(None) => {
                        info!("GATT event:: **NONE** processed without specific action");
                    }
                    Err(e) => {
                        info!("Error processing GATT event: {:?}", e);
                    }
                }
            }
            ConnectionEvent::Disconnected { reason } => {
                info!("Client disconnected: {:?}", reason);
                break;
            }
            _ => {}
        }

        // Optional: Periodic notification to keep connection alive
        Timer::after(Duration::from_millis(100)).await;
    }

    info!("BLE Task Ended");
}

async fn ble_task_bk<C: Controller>(
    server: &Server<'_>,
    connection: Connection<'_>,
    stack: Stack<'_, C>,
) {
    let random = server.random_service.random_value;
    let mut i = 0;

    // Create a clone of the connection for events
    let conn_for_events = connection.clone();

    loop {
        info!("Muy loco aqui el loop");
        match conn_for_events.next().await {
            ConnectionEvent::Disconnected { reason } => {
                info!("Client disconnected: {:?}", reason);
                break;
            }
            ConnectionEvent::Gatt { data } => {
                info!("GATT eventl");
            }
            event => {
                info!("Received event");
            }
        }
    }

    // let _ = embassy_futures::join::join(
    //     // Task to handle connection events
    //     async move {
    //         loop {
    //             info!("Muy loco aqui el loop");
    //             match conn_for_events.next().await {
    //                 ConnectionEvent::Disconnected { reason } => {
    //                     info!("Client disconnected: {:?}", reason);
    //                     break;
    //                 }
    //                 ConnectionEvent::Gatt { data } => {
    //                     info!("GATT eventl");
    //                 }
    //                 event => {
    //                     info!("Received event");
    //                 }
    //             }
    //         }
    //     },
    //     // Task to send notifications
    //     async move {
    //         loop {
    //             let value: u8 = i;
    //             if i > 50 {
    //                 i = 0;
    //             }
    //             i += 1;

    //             match random.notify(server, &connection, &value).await {
    //                 Ok(_) => {
    //                     info!("Notified random value: {}", value);
    //                 }
    //                 Err(e) => {
    //                     info!("Error notifying value: {:?}", e);
    //                     break;
    //                 }
    //             }

    //             Timer::after(Duration::from_secs(10)).await;
    //         }
    //     },
    // )
    // .await;
}

async fn run_ble(controller: impl Controller) {
    let mut resources = HostResources::<_, 1, 2, 251>::new(PacketQos::None);

    let address = Address::random([0xE5, 0x80, 0x16, 0x79, 0xCF, 0x5A]);

    info!("Setting BLE address to {:?}", address);
    let (stack, mut peripheral, _, mut runner) = trouble_host::new(controller, &mut resources)
        .set_random_address(address)
        .build();

    info!("Starting advertising and GATT service");

    // Initialize GATT server
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "ELOY VBT",
        appearance: &appearance::UNKNOWN,
    }))
    .unwrap();

    // Use embassy_futures::join to run both the runner and advertising tasks
    let _ = embassy_futures::join::join(
        // Runner task - this is essential!
        runner.run(),
        // Our advertising task
        async move {
            loop {
                let mut adv_data = [0; 31];
                let mut scan_data = [0; 31];

                let len = AdStructure::encode_slice(
                    &[
                        AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                        AdStructure::ServiceUuids16(&[Uuid::Uuid16([0x12, 0x34])]),
                        AdStructure::CompleteLocalName(b"Eloy VBT"),
                    ],
                    &mut adv_data[..],
                )
                .unwrap();

                let scan_len = AdStructure::encode_slice(
                    &[AdStructure::CompleteLocalName(b"Eloy VBT")],
                    &mut scan_data[..],
                )
                .unwrap();

                let params = AdvertisementParameters {
                    interval_min: Duration::from_millis(30),
                    interval_max: Duration::from_millis(60),
                    tx_power: TxPower::Plus8dBm,
                    ..Default::default()
                };

                info!(
                    "Starting advertisement with parameters: interval={:?}-{:?}",
                    params.interval_min, params.interval_max
                );

                let advertiser = match peripheral
                    .advertise(
                        &params,
                        Advertisement::ConnectableScannableUndirected {
                            adv_data: &adv_data[..len],
                            scan_data: &scan_data[..scan_len],
                        },
                    )
                    .await
                {
                    Ok(advertiser) => {
                        info!("Advertising started with data length {} bytes", len);
                        advertiser
                    }
                    Err(e) => {
                        info!("Failed to start advertising: {:?}", e);
                        continue;
                    }
                };

                match advertiser.accept().await {
                    Ok(connection) => {
                        info!("Client connected!");
                        ble_task(&server, connection, stack).await;
                    }
                    Err(e) => {
                        info!("Error accepting connection: {:?}", e);
                    }
                }

                Timer::after(Duration::from_millis(100)).await;
            }
        },
    )
    .await;
}

// async fn run_ble(controller: impl Controller) {
//     let mut resources = HostResources::<_, 1, 2, 251>::new(PacketQos::None);

//     let address = Address::random([0xE5, 0x80, 0x16, 0x79, 0xCF, 0x5A]);

//     info!("Setting BLE address to {:?}", address);
//     let (stack, mut peripheral, _, runner) = trouble_host::new(controller, &mut resources)
//         .set_random_address(address)
//         .build();

//     info!("Starting advertising and GATT service");

//     // Initialize GATT server
//     let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
//         name: "ELOY VBT",
//         appearance: &appearance::UNKNOWN,
//     }))
//     .unwrap();

//     loop {
//         // Prepare advertisement data
//         let mut adv_data = [0; 31];
//         let mut scan_data = [0; 31];

//         let len = AdStructure::encode_slice(
//             &[
//                 AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
//                 // Add service UUID to help with discovery
//                 AdStructure::ServiceUuids16(&[Uuid::Uuid16([0x18, 0x0A])]), // 0x180A in bytes
//                 AdStructure::CompleteLocalName(b"Eloy VBT"),
//             ],
//             &mut adv_data[..],
//         )
//         .unwrap();
//         info!("Advertisement data encoded with length: {}", len);
//         info!("Starting advertising with address {:?}", address);
//         let params = AdvertisementParameters {
//             // Use shorter intervals for better discovery
//             interval_min: Duration::from_millis(30),
//             interval_max: Duration::from_millis(60),
//             // Increase TX power for better range
//             tx_power: TxPower::Plus8dBm,
//             ..Default::default()
//         };
//         let scan_len = AdStructure::encode_slice(
//             &[AdStructure::CompleteLocalName(b"Eloy VBT")],
//             &mut scan_data[..],
//         )
//         .unwrap();
//         let advertiser = match peripheral
//             .advertise(
//                 &params,
//                 Advertisement::ConnectableScannableUndirected {
//                     adv_data: &adv_data[..len],
//                     scan_data: &scan_data[..scan_len],
//                 },
//             )
//             .await
//         {
//             Ok(advertiser) => {
//                 info!("Advertising started with data length {} bytes", len);
//                 advertiser
//             }
//             Err(e) => {
//                 info!("Failed to start advertising: {:?}", e);
//                 return;
//             }
//         };
//         info!("Advertising started successfully");
//         info!("BLE stack initialized");
//         info!("Radio status: OK");
//         info!(
//             "Starting advertisement with parameters: interval={:?}-{:?}",
//             params.interval_min, params.interval_max
//         );
//         // Wait for connection
//         match advertiser.accept().await {
//             Ok(connection) => {
//                 info!("Client connected!");
//                 ble_task(&server, connection, stack).await;
//                 // // Run the notification task
//                 // ble_task(server, connection, stack).await;
//             }
//             Err(e) => {
//                 info!("Error accepting connection: {:?}", e);
//             }
//         }
//         info!("Advertising finished??");
//     }
// }

#[main]
async fn main(spawner: Spawner) {
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
    run_ble(controller).await;

    loop {
        info!("Hello world!");
        Timer::after(Duration::from_secs(60)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/v0.22.0/examples/src/bin
}
