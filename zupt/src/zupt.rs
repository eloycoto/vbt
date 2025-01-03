#![no_std]
#![allow(warnings)]

use crate::stadistics::Median;
use crate::utils::median_stddev;
use crate::utils::rotate_vector_with_angle;

use heapless::Vec;
use micromath::statistics::StdDev;
use micromath::vector::{F32x3, Vector3d};
use micromath::F32Ext;

pub const MAX_CALIBRATION_SAMPLES: usize = 32;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MovementState {
    Waiting,
    UpwardMovement,
    PeakDetected,
}

#[derive(Debug, Clone, Copy)]
pub enum MovementEventType {
    LowerPoint,
    Peak,
    None,
}

#[derive(Debug, Clone, Copy)]
struct MovementPoint {
    position: F32x3,
    acceleration: f32,
    time: f32,
}

pub struct MovementDetector {
    // Configuration
    dt: f32,
    up_threshold: f32,

    // Calibration
    baseline: F32x3,
    rotation_angle: f32,
    noise_level: f32,
    r: f32,

    // State tracking
    state: MovementState,
    lowest_point: MovementPoint,
    peak_point: MovementPoint,
    current_time: f32,
}

impl MovementDetector {
    pub fn new() -> Self {
        Self {
            dt: 0.1,
            up_threshold: 0.2,
            baseline: F32x3::default(),
            rotation_angle: 0.0,
            noise_level: 0.0,
            r: 0.0,
            state: MovementState::Waiting,
            lowest_point: MovementPoint {
                position: F32x3::default(),
                acceleration: 0.0,
                time: 0.0,
            },
            peak_point: MovementPoint {
                position: F32x3::default(),
                acceleration: 0.0,
                time: 0.0,
            },
            current_time: 0.0,
        }
    }

    // The need for calibration, to detect the noise and the rotation of the accelerometer based on
    // the quatarnios.
    pub fn calibrate(&mut self, samples: &[F32x3]) {
        if samples.is_empty() {
            return;
        }

        let mut total_xy = 0.0;
        let mut total_z = 0.0;

        for sample in samples {
            let xy_magnitude = (sample.x * sample.x + sample.y * sample.y).sqrt();
            total_xy += xy_magnitude;
            total_z += sample.z.abs();
        }

        let avg_xy = total_xy / samples.len() as f32;
        let avg_z = total_z / samples.len() as f32;
        self.rotation_angle = avg_xy.atan2(avg_z);

        let mut rotated_sum = F32x3::default();
        let mut rotated_z_values: heapless::Vec<f32, 32> = heapless::Vec::new();
        let mut rotated_values: heapless::Vec<F32x3, 32> = heapless::Vec::new();
        for sample in samples {
            let rotated_vector = self.rotate_vector(*sample);
            rotated_z_values.push(rotated_vector.z).ok();
            rotated_sum = rotated_sum + rotated_vector;
            rotated_values.push(rotated_vector);
        }

        self.baseline = rotated_values.as_slice().median::<16>();
        self.noise_level = median_stddev(rotated_z_values.as_slice());
        self.r = self.noise_level * self.noise_level;
    }

    /// Rotate a vector using the calibrated rotation angle
    fn rotate_vector(&self, v: F32x3) -> F32x3 {
        rotate_vector_with_angle(v, self.rotation_angle)
    }

    /// Process a new sample and detect movements
    pub fn process_sample(&mut self, sample: F32x3) -> (bool, MovementEventType, f32) {
        // Rotate the sample
        let rotated = self.rotate_vector(sample);

        // Calculate vertical acceleration
        let current_accel = rotated.z - self.baseline.z;

        // Update time
        self.current_time += self.dt;

        match self.state {
            MovementState::Waiting => {
                if current_accel < self.lowest_point.acceleration {
                    self.lowest_point = MovementPoint {
                        position: rotated,
                        acceleration: current_accel,
                        time: self.current_time,
                    };

                    if current_accel > self.up_threshold {
                        self.state = MovementState::UpwardMovement;
                    }

                    return (true, MovementEventType::LowerPoint, 0.0);
                }
            }

            MovementState::UpwardMovement => {
                if current_accel > self.peak_point.acceleration {
                    let time_diff = self.current_time - self.lowest_point.time;

                    if time_diff > 0.3 {
                        // Minimum time threshold
                        let z_diff = rotated.z - self.lowest_point.position.z;
                        let velocity = z_diff / time_diff;

                        self.peak_point = MovementPoint {
                            position: rotated,
                            acceleration: current_accel,
                            time: self.current_time,
                        };

                        self.state = MovementState::PeakDetected;
                        return (true, MovementEventType::Peak, velocity);
                    }
                }
            }

            MovementState::PeakDetected => {
                if current_accel < -0.1 {
                    // Reset for next movement
                    if self.current_time - self.peak_point.time > 0.4 {
                        self.state = MovementState::Waiting;
                        self.lowest_point.acceleration = current_accel;
                    }
                }
            }
        }

        (false, MovementEventType::None, 0.0)
    }
}

#[cfg(test)]
mod tests {
    use micromath::vector::Vector;
    use micromath::F32Ext;

    use super::*;

    fn get_samples() -> [F32x3; 10] {
        [
            F32x3::from_slice(&[5.0, -58.0, 221.0]),
            F32x3::from_slice(&[-7.0, -58.0, 239.0]),
            F32x3::from_slice(&[-3.0, -65.0, 229.0]),
            F32x3::from_slice(&[-5.0, -65.0, 231.0]),
            F32x3::from_slice(&[0.0, -62.0, 231.0]),
            F32x3::from_slice(&[0.0, -64.0, 232.0]),
            F32x3::from_slice(&[-3.0, -63.0, 235.0]),
            F32x3::from_slice(&[0.0, -66.0, 233.0]),
            F32x3::from_slice(&[0.0, -65.0, 233.0]),
            F32x3::from_slice(&[-1.0, -64.0, 232.0]),
        ]
    }

    #[test]
    fn test_basic_movement() {
        let mut detector = MovementDetector::new();
        let samples = get_samples();

        detector.calibrate(&samples);
        assert_eq!(detector.noise_level, 4.3817806);
        assert_eq!(detector.rotation_angle, 0.26598683);
        assert_eq!(detector.r, 19.2);
        assert_eq!(detector.baseline.x, 16.07786);
        assert_eq!(detector.baseline.y, -61.88078);
        assert_eq!(detector.baseline.z, 232.0);
    }
}
