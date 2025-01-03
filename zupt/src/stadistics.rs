use micromath::vector::{Component, Vector3d};

pub trait Median {
    type Output;
    fn median<const N: usize>(&self) -> Self::Output;
}

impl<C> Median for &[Vector3d<C>]
where
    C: Component + Into<f32> + Copy + PartialOrd,
{
    type Output = Vector3d<f32>;

    fn median<const N: usize>(&self) -> Vector3d<f32> {
        if self.is_empty() {
            return Vector3d::default();
        }

        // Create separate vectors for x, y, z components
        let mut x_values: heapless::Vec<f32, N> = heapless::Vec::new();
        let mut y_values: heapless::Vec<f32, N> = heapless::Vec::new();
        let mut z_values: heapless::Vec<f32, N> = heapless::Vec::new();

        // Extract components
        for v in *self {
            x_values.push(v.x.into()).ok();
            y_values.push(v.y.into()).ok();
            z_values.push(v.z.into()).ok();
        }

        // Sort each component vector
        x_values.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        y_values.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        z_values.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        let len = self.len();
        let mid = len / 2;

        // Calculate median for each component
        let (median_x, median_y, median_z) = if len % 2 == 0 {
            // Even number of elements - average the middle two
            (
                (x_values[mid - 1] + x_values[mid]) / 2.0,
                (y_values[mid - 1] + y_values[mid]) / 2.0,
                (z_values[mid - 1] + z_values[mid]) / 2.0,
            )
        } else {
            // Odd number of elements - take the middle element
            (x_values[mid], y_values[mid], z_values[mid])
        };

        Vector3d {
            x: median_x,
            y: median_y,
            z: median_z,
        }
    }
}

// For array implementation
impl<C, const ARRAY_SIZE: usize> Median for [Vector3d<C>; ARRAY_SIZE]
where
    C: Component + Into<f32> + Copy + PartialOrd,
{
    type Output = Vector3d<f32>;

    fn median<const N: usize>(&self) -> Vector3d<f32> {
        self.as_slice().median::<N>()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use micromath::vector::{F32x3, Vector};

    #[test]
    fn test_median_odd() {
        let vectors = [
            F32x3 {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
            F32x3 {
                x: 4.0,
                y: 5.0,
                z: 6.0,
            },
            F32x3 {
                x: 7.0,
                y: 8.0,
                z: 9.0,
            },
        ];

        let median = vectors.as_slice().median::<16>();
        assert_eq!(median.x, 4.0);
        assert_eq!(median.y, 5.0);
        assert_eq!(median.z, 6.0);
    }

    #[test]
    fn test_median_even() {
        let vectors = [
            F32x3 {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
            F32x3 {
                x: 4.0,
                y: 5.0,
                z: 6.0,
            },
            F32x3 {
                x: 7.0,
                y: 8.0,
                z: 9.0,
            },
            F32x3 {
                x: 10.0,
                y: 11.0,
                z: 12.0,
            },
        ];

        let median = vectors.as_slice().median::<16>();
        assert_eq!(median.x, 5.5);
        assert_eq!(median.y, 6.5);
        assert_eq!(median.z, 7.5);
    }

    #[test]
    fn test_real_data() {
        let vectors = [
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
        ];

        let median = vectors.as_slice().median::<16>();
        assert_eq!(median.x, -0.5);
        assert_eq!(median.y, -64.0);
        assert_eq!(median.z, 232.0);
    }
}
