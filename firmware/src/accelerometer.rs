#![allow(warnings)]
use core::fmt::Write;
use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c::I2c;
use heapless::Vec;
use log::info;
use micromath::F32Ext;

const ID_VAL: u8 = 0b11100101;
const EARTH_GRAVITY: f32 = 9.80665;
const LSB_SCALE_FACTOR_FULL_RES: f32 = 0.0039;

const ADXL345_ADDR: u8 = 0x53;
const ADXL345_POWER_CTL: u8 = 0x2D;
const ADXL345_DATA_FORMAT: u8 = 0x31;
const ADXL345_DATAX0: u8 = 0x32;
const ADXL345_DATAX1: u8 = 0x33;
const ADXL345_DATAY0: u8 = 0x34;
const ADXL345_DATAY1: u8 = 0x35;
const ADXL345_DATAZ0: u8 = 0x36;
const ADXL345_DATAZ1: u8 = 0x37;

const WINDOW_SIZE: usize = 5;
const CALIBRATION_SAMPLES: usize = 10;
pub struct MotionDetector<I2C> {
    i2c: I2C,
    baseline: Option<(f32, f32, f32)>,
    samples: [(f32, f32, f32); WINDOW_SIZE],
    sample_index: usize,
    sample_count: usize,
    motion_threshold: f32,
    is_moving: bool,
    last_position: DevicePosition,
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
            motion_threshold: 0.1,
            is_moving: false,
            last_position: DevicePosition::Unknown,
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

        Ok(())
    }

    pub async fn get_motion_state(&mut self) -> Result<MotionState, I2C::Error> {
        let raw_accel = self.get_accel().await?;

        self.samples[self.sample_index] = raw_accel;
        self.sample_index = (self.sample_index + 1) % WINDOW_SIZE;
        if self.sample_count < WINDOW_SIZE {
            self.sample_count += 1;
        }

        let avg_accel = self.calculate_moving_average();

        let relative_accel = match self.baseline {
            Some(baseline) => (
                avg_accel.0 - baseline.0,
                avg_accel.1 - baseline.1,
                avg_accel.2 - baseline.2,
            ),
            None => avg_accel,
        };

        let acceleration_magnitude =
            (relative_accel.0.powi(2) + relative_accel.1.powi(2) + relative_accel.2.powi(2)).sqrt();

        self.is_moving = acceleration_magnitude > self.motion_threshold;

        // Determine position based on z-axis
        let new_position = if relative_accel.2 > 0.5 {
            DevicePosition::Down
        } else if relative_accel.2 < -0.5 {
            DevicePosition::Up
        } else {
            self.last_position.clone()
        };

        self.last_position = new_position.clone();

        Ok(MotionState {
            position: new_position,
            is_moving: self.is_moving,
            acceleration: avg_accel,
            relative_acceleration: relative_accel,
        })
    }

    fn calculate_moving_average(&self) -> (f32, f32, f32) {
        let mut sum = (0.0, 0.0, 0.0);
        let count = self.sample_count;

        // Only average the number of samples we've actually collected
        for i in 0..count {
            sum.0 += self.samples[i].0;
            sum.1 += self.samples[i].1;
            sum.2 += self.samples[i].2;
        }

        (
            sum.0 / count as f32,
            sum.1 / count as f32,
            sum.2 / count as f32,
        )
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum DevicePosition {
    Up,
    Down,
    Unknown,
}

#[derive(Debug)]
pub struct MotionState {
    pub position: DevicePosition,
    pub is_moving: bool,
    pub acceleration: (f32, f32, f32),
    pub relative_acceleration: (f32, f32, f32),
}

impl MotionState {
    pub fn is_up(&self) -> bool {
        self.position == DevicePosition::Up
    }

    pub fn is_down(&self) -> bool {
        self.position == DevicePosition::Down
    }

    pub fn get_acceleration(&self) -> (f32, f32, f32) {
        self.acceleration
    }

    pub fn debug_str(&self) -> heapless::String<128> {
        let mut s = heapless::String::new();
        let _ = write!(
            s,
            "M{{pos:{:?},mov:{}}} acceleration: {:?}, relative_acceleration: {:?}",
            self.position, self.is_moving, self.acceleration, self.relative_acceleration
        );
        s
    }
}
