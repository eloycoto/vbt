use embedded_hal_async::i2c::I2c;
use micromath::F32Ext;

const EARTH_GRAVITY: f32 = 9.80665;
const LSB_SCALE_FACTOR_FULL_RES: f32 = 0.0039;

const ADXL345_ADDR: u8 = 0x53;
const ADXL345_POWER_CTL: u8 = 0x2D;
// const ADXL345_DATA_FORMAT: u8 = 0x31;
const ADXL345_DATAX0: u8 = 0x32;
// const ADXL345_DATAX1: u8 = 0x33;
// const ADXL345_DATAY0: u8 = 0x34;
// const ADXL345_DATAY1: u8 = 0x35;
// const ADXL345_DATAZ0: u8 = 0x36;
// const ADXL345_DATAZ1: u8 = 0x37;

const CALIBRATION_SAMPLES: usize = 10;

const SAMPLING_RATE: f32 = 10.0; // Hz - for 100ms sampling period
const WINDOW_SIZE: usize = 60;

pub struct MotionDetector<I2C> {
    i2c: I2C,
    baseline: Option<(f32, f32, f32)>,
    samples: [(f32, f32, f32); WINDOW_SIZE],
    sample_index: usize,
    sample_count: usize,
    last_position: LiftPosition,
}

impl<I2C> MotionDetector<I2C>
where
    I2C: I2c,
{
    pub async fn new(i2c: I2C) -> Result<Self, I2C::Error> {
        let mut detector = Self {
            i2c,
            baseline: None,
            samples: [(0.0, 0.0, 0.0); WINDOW_SIZE],
            sample_index: 0,
            sample_count: 0,
            last_position: LiftPosition::Rest,
        };
        detector.power_on().await?;
        detector.calibrate().await?;
        Ok(detector)
    }

    pub async fn get_accel_raw(&mut self) -> Result<(i16, i16, i16), I2C::Error> {
        let tx_buf = [ADXL345_DATAX0; 1];
        let mut rx_buf = [0u8; 6]; // Read all axes
        self.i2c
            .write_read(ADXL345_ADDR, &tx_buf, &mut rx_buf)
            .await?;

        let x = i16::from_le_bytes([rx_buf[0], rx_buf[1]]);
        let y = i16::from_le_bytes([rx_buf[2], rx_buf[3]]);
        let z = i16::from_le_bytes([rx_buf[4], rx_buf[5]]);

        Ok((x, y, z))
    }

    pub async fn get_accel(&mut self) -> Result<(f32, f32, f32), I2C::Error> {
        let accel = self.get_accel_raw().await?;
        let accel_g: (f32, f32, f32) = (
            (accel.0 as f32) * EARTH_GRAVITY * LSB_SCALE_FACTOR_FULL_RES,
            (accel.1 as f32) * EARTH_GRAVITY * LSB_SCALE_FACTOR_FULL_RES,
            (accel.2 as f32) * EARTH_GRAVITY * LSB_SCALE_FACTOR_FULL_RES,
        );

        Ok(accel_g)
    }

    async fn power_on(&mut self) -> Result<(), I2C::Error> {
        let tx_buf = [ADXL345_POWER_CTL, 0x08];
        self.i2c.write(ADXL345_ADDR, &tx_buf).await?;
        Ok(())
    }

    pub async fn calibrate(&mut self) -> Result<(), I2C::Error> {
        let mut sum = (0.0f32, 0.0f32, 0.0f32);

        for _ in 0..CALIBRATION_SAMPLES {
            let accel = self.get_accel().await?;
            sum.0 += accel.0;
            sum.1 += accel.1;
            sum.2 += accel.2;
        }

        // Calculate average as baseline
        self.baseline = Some((
            sum.0 / CALIBRATION_SAMPLES as f32,
            sum.1 / CALIBRATION_SAMPLES as f32,
            sum.2 / CALIBRATION_SAMPLES as f32,
        ));
        log::info!("Calibration complete. Baseline: {:?}", self.baseline);
        Ok(())
    }
    pub async fn get_next_motion_state(&mut self) -> Result<MotionState, I2C::Error> {
        // Get current acceleration reading
        let accel = self.get_accel().await?;
        let z_accel = accel.2;

        // Get the baseline-adjusted acceleration
        let adjusted_accel = match self.baseline {
            Some(baseline) => z_accel - baseline.2,
            None => z_accel,
        };

        // Update sliding window of samples
        self.samples[self.sample_index] = (accel.0, accel.1, adjusted_accel);
        self.sample_index = (self.sample_index + 1) % WINDOW_SIZE;
        if self.sample_count < WINDOW_SIZE {
            self.sample_count += 1;
        }

        // Calculate velocity through integration
        let dt = 1.0 / SAMPLING_RATE; // 0.1 seconds for 10Hz
        let velocity = self.calculate_velocity(adjusted_accel, dt);

        // Determine motion state based on acceleration and velocity
        let position = self.determine_position(adjusted_accel, velocity);

        Ok(MotionState {
            position,
            velocity,
            acceleration: adjusted_accel,
        })
    }

    fn calculate_velocity(&self, _: f32, dt: f32) -> f32 {
        if self.sample_count < 2 {
            return 0.0;
        }

        // Use a moving average of acceleration for smoother velocity
        let mut sum_accel = 0.0;
        let mut count = 0;

        for i in 0..self.sample_count {
            sum_accel += self.samples[i].2;
            count += 1;
        }

        let avg_accel = if count > 0 {
            sum_accel / count as f32
        } else {
            0.0
        };
        avg_accel * dt // Simple integration for velocity
    }

    fn determine_position(&mut self, accel: f32, velocity: f32) -> LiftPosition {
        // Adjusted thresholds based on observed values
        const VELOCITY_THRESHOLD: f32 = 0.05; // m/s
        const ACCELERATION_THRESHOLD: f32 = 0.1; // m/s²

        // Store the current position before updating
        let current_position = match (
            velocity.abs() < VELOCITY_THRESHOLD,
            accel.abs() < ACCELERATION_THRESHOLD,
        ) {
            (true, true) => {
                if self.last_position == LiftPosition::Up {
                    LiftPosition::TopPosition
                } else if self.last_position == LiftPosition::Down {
                    LiftPosition::BottomPosition
                } else {
                    LiftPosition::Rest
                }
            }

            _ => {
                if velocity > VELOCITY_THRESHOLD {
                    LiftPosition::MovingUp
                } else if velocity < -VELOCITY_THRESHOLD {
                    LiftPosition::MovingDown
                } else {
                    match self.last_position {
                        LiftPosition::Up => LiftPosition::TopPosition,
                        LiftPosition::Down => LiftPosition::BottomPosition,
                        _ => LiftPosition::Rest,
                    }
                }
            }
        };

        self.last_position = match current_position {
            LiftPosition::TopPosition => LiftPosition::Up,
            LiftPosition::BottomPosition => LiftPosition::Down,
            _ => self.last_position,
        };

        current_position
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LiftPosition {
    Rest,
    PREVIOUS,
    MovingUp,
    MovingDown,
    TopPosition,
    BottomPosition,
    Up,
    Down,
}

#[derive(Debug, Clone)]
pub struct MotionState {
    pub position: LiftPosition,
    pub velocity: f32,     // Current velocity in m/s
    pub acceleration: f32, // Current acceleration in m/s²
}

impl MotionState {
    pub fn is_up(&self) -> bool {
        self.position == LiftPosition::MovingUp
    }

    pub fn is_down(&self) -> bool {
        self.position == LiftPosition::MovingDown
    }

    // pub fn get_acceleration(&self) -> (f32, f32, f32) {
    //     self.acceleration
    // }

    // pub fn debug_str(&self) -> heapless::String<128> {
    //     let mut s = heapless::String::new();
    //     let _ = write!(
    //         s,
    //         "Position: {}",
    //         self.position, self.is_moving, self.acceleration.2, self.relative_acceleration.2
    //     );
    //     s
    // }
}
