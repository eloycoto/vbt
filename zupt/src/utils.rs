use micromath::vector::F32x3;

// Get a vector and rotated based on a given angle.
pub fn rotate_vector_with_angle(v: F32x3, rotation_angle: f32) -> F32x3 {
    let cos_angle = rotation_angle.cos();
    let sin_angle = rotation_angle.sin();

    F32x3 {
        x: v.x * cos_angle - v.y * sin_angle,
        y: v.x * sin_angle + v.y * cos_angle,
        z: v.z,
    }
}

// Micromath use mean instead of median, with noise values, it's much better mean because clean
// better the data.
pub fn median_stddev(values: &[f32]) -> f32 {
    if values.is_empty() {
        return 0.0;
    }

    let mut sorted_values = values.to_vec();
    sorted_values.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let median = sorted_values[sorted_values.len() / 2];

    // Calculate squared differences from the median
    let variance = sorted_values
        .iter()
        .map(|&x| (x - median).powi(2))
        .sum::<f32>()
        / (sorted_values.len() as f32);

    variance.sqrt()
}
