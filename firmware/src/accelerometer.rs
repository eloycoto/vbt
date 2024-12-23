#![allow(warnings)]
use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c::I2c;
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

pub struct MotionDetector<I2C> {
    i2c: I2C,
}

impl<I2C> MotionDetector<I2C>
where
    I2C: I2c,
{
    pub async fn new(i2c: I2C) -> Result<Self, I2C::Error> {
        let mut detector = Self { i2c };
        detector.power_on().await?;
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
}
