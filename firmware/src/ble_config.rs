#![allow(warnings)]
use embassy_futures::join;
use embassy_time::{Duration, Timer};
use heapless::Vec;
use log::{debug, error, info, warn};
use trouble_host::prelude::*;

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

/// Maximum number of notification subscribers
const MAX_SUBSCRIBERS: usize = 8;

/// Represents the current state of the BLE connection
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BleConnectionState {
    Disconnected,
    Advertising,
    Connected,
    Error(BleError),
}

/// Custom error type for BLE operations
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BleError {
    AdvertisingFailed,
    ConnectionFailed,
    NotificationFailed,
}

/// Configuration for the BLE controller
#[derive(Clone)]
pub struct BleConfig {
    pub device_name: &'static str,
    pub service_uuid: u16,
    pub min_interval: Duration,
    pub max_interval: Duration,
    pub tx_power: TxPower,
}

impl Default for BleConfig {
    fn default() -> Self {
        Self {
            device_name: "Eloy VBT",
            service_uuid: 0x1234,
            min_interval: Duration::from_millis(30),
            max_interval: Duration::from_millis(60),
            tx_power: TxPower::Plus8dBm,
        }
    }
}

pub struct BleController {
    config: BleConfig,
    connection_state: BleConnectionState,
    notification_subscribers: Vec<fn(BleConnectionState), MAX_SUBSCRIBERS>,
}

impl BleController {
    pub fn new(config: BleConfig) -> Self {
        Self {
            config,
            connection_state: BleConnectionState::Disconnected,
            notification_subscribers: Vec::new(),
        }
    }

    pub fn subscribe_to_state_changes(
        &mut self,
        callback: fn(BleConnectionState),
    ) -> Result<(), ()> {
        match self.notification_subscribers.push(callback) {
            Ok(_) => {
                debug!("Added new state change subscriber");
                Ok(())
            }
            Err(_) => {
                warn!("Failed to add subscriber: maximum subscribers reached");
                Err(())
            }
        }
    }

    fn update_state(&mut self, new_state: BleConnectionState) {
        self.connection_state = new_state;
        for subscriber in self.notification_subscribers.iter() {
            subscriber(new_state);
        }
        info!("BLE state changed to: {:?}", new_state);
    }

    /// Gets the current connection state
    pub fn get_state(&self) -> BleConnectionState {
        self.connection_state
    }

    /// Starts the BLE controller with the given hardware controller
    pub async fn run<C: Controller>(&mut self, controller: C) {
        info!("*****Starting BLE controller*************");

        let mut resources = HostResources::<_, 1, 2, 251>::new(PacketQos::None);
        let address = Address::random([0xE5, 0x80, 0x16, 0x79, 0xCF, 0x5A]);

        info!("Setting BLE address to {:?}", address);

        let (stack, mut peripheral, _, mut runner) = trouble_host::new(controller, &mut resources)
            .set_random_address(address)
            .build();

        info!("RUN --->2");
        let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
            name: self.config.device_name,
            appearance: &appearance::UNKNOWN,
        }))
        .unwrap();
        info!("RUN --->1");
        let _ = join::join(
            runner.run(),
            self.run_advertising_loop(&mut peripheral, &server, stack),
        )
        .await;
    }

    /// Runs the advertising loop
    async fn run_advertising_loop<C: Controller>(
        &mut self,
        peripheral: &mut Peripheral<'_, C>,
        server: &Server<'_>,
        stack: Stack<'_, C>,
    ) {
        loop {
            let mut adv_data = [0; 31];
            let mut scan_data = [0; 31];
            let len = AdStructure::encode_slice(
                &[
                    AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                    AdStructure::ServiceUuids16(&[Uuid::Uuid16(
                        self.config.service_uuid.to_be_bytes(),
                    )]),
                    AdStructure::CompleteLocalName(self.config.device_name.as_bytes()),
                ],
                &mut adv_data[..],
            )
            .unwrap();

            let scan_len = AdStructure::encode_slice(
                &[AdStructure::CompleteLocalName(
                    self.config.device_name.as_bytes(),
                )],
                &mut scan_data[..],
            )
            .unwrap();

            let params = AdvertisementParameters {
                interval_min: self.config.min_interval,
                interval_max: self.config.max_interval,
                tx_power: self.config.tx_power,
                ..Default::default()
            };

            self.update_state(BleConnectionState::Advertising);
            info!(
                "Starting advertisement with parameters: interval={:?}-{:?}",
                params.interval_min, params.interval_max
            );

            match peripheral
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
                    match advertiser.accept().await {
                        Ok(connection) => {
                            info!("Client connected!");
                            self.update_state(BleConnectionState::Connected);
                            self.handle_connection(server, connection, stack.clone())
                                .await;
                        }
                        Err(e) => {
                            error!("Error accepting connection: {:?}", e);
                            self.update_state(BleConnectionState::Error(
                                BleError::ConnectionFailed,
                            ));
                        }
                    }
                }
                Err(e) => {
                    error!("Failed to start advertising: {:?}", e);
                    self.update_state(BleConnectionState::Error(BleError::AdvertisingFailed));
                }
            }

            Timer::after(Duration::from_millis(100)).await;
        }
    }

    /// Handles an active connection
    async fn handle_connection<C: Controller>(
        &mut self,
        server: &Server<'_>,
        connection: Connection<'_>,
        stack: Stack<'_, C>,
    ) {
        while let event = connection.next().await {
            match event {
                ConnectionEvent::Gatt { data } => {
                    debug!("GATT event received: {:?}", data.request());
                    match data.process(server).await {
                        Ok(Some(event)) => match event {
                            GattEvent::Read(read_event) => {
                                debug!("Read event for handle: {}", read_event.handle());
                                let _ = read_event.reply(Ok(())).await;
                            }
                            GattEvent::Write(write_event) => {
                                debug!("Write event for handle: {}", write_event.handle());
                                let _ = write_event.reply(Ok(())).await;
                            }
                        },
                        Ok(None) => {
                            debug!("GATT event processed without specific action");
                        }
                        Err(e) => {
                            error!("Error processing GATT event: {:?}", e);
                        }
                    }
                }
                ConnectionEvent::Disconnected { reason } => {
                    info!("Client disconnected: {:?}", reason);
                    self.update_state(BleConnectionState::Disconnected);
                    break;
                }
            }
            Timer::after(Duration::from_millis(100)).await;
        }
    }
}
