import numpy as np
from scipy.linalg import inv
from typing import List, Tuple

def calculate_rotation(samples: List[Tuple[float, float, float]]) -> float:
    """
    Calculate rotation angle from calibration samples by analyzing the x-y plane.
    Assumes the sensor should ideally be aligned with the Y axis when standing still.

    Args:
        samples: List of (x, y, z) calibration measurements

    Returns:
        angle: Rotation angle in radians
    """
    # Convert samples to numpy array for easier processing
    samples_array = np.array(samples)

    # Calculate mean x and y values
    mean_x = np.mean(samples_array[:, 0])
    mean_y = np.mean(samples_array[:, 1])

    # Calculate rotation angle using arctan2
    # arctan2 handles all quadrants correctly
    rotation_angle = np.arctan2(mean_x, mean_y)

    return rotation_angle

def rotate_coordinates(x: float, y: float, z: float, angle: float) -> Tuple[float, float, float]:
    """
    Rotate coordinates around the Z axis by given angle.

    Args:
        x, y, z: Original coordinates
        angle: Rotation angle in radians

    Returns:
        Tuple of rotated (x, y, z) coordinates
    """
    # Create rotation matrix around Z axis
    cos_angle = np.cos(angle)
    sin_angle = np.sin(angle)

    rotation_matrix = np.array([
        [cos_angle, -sin_angle, 0],
        [sin_angle, cos_angle, 0],
        [0, 0, 1]
    ])

    # Apply rotation
    rotated_coords = rotation_matrix @ np.array([x, y, z])

    return tuple(rotated_coords)

class JumpDetector:
    def __init__(self, calibration_samples: List[Tuple[float, float, float]] = None):

        self.samples = []

        self.dt = 0.1  # 100ms sampling

        # State vector: [position, velocity]
        self.state = np.zeros(2)

        # State transition matrix
        self.F = np.array([[1, self.dt],
                          [0, 1]])

        # Process noise - tuned for the observed data
        self.Q = np.array([[0.5, 0],
                          [0, 1.0]])

        # Measurement noise
        self.R = np.array([[25.0]])  # High due to observed noise

        # State covariance
        self.P = np.array([[1, 0],
                          [0, 1]])

        # Will be set during calibration
        self.baseline_z = None
        self.baseline_x = None
        self.baseline_y = None

        # Jump detection parameters
        self.jump_threshold = 0.5  # Tuned based on your data
        self.landing_threshold = -0.6
        self.in_jump = False

        if calibration_samples:
            self.calibrate(calibration_samples)

    def calibrate(self, samples: List[Tuple[float, float, float]]):
      """Calculate baseline values and rotation from static samples"""
      # Calculate rotation first
      self.rotation_angle = calculate_rotation(samples)

      # Rotate all calibration samples
      rotated_samples = [
          rotate_coordinates(x, y, z, self.rotation_angle)
          for x, y, z in samples
      ]
      rotated_array = np.array(rotated_samples)

      # Calculate median values as baselines (more robust than mean)
      self.baseline_x = np.median(rotated_array[:, 0])
      self.baseline_y = np.median(rotated_array[:, 1])
      self.baseline_z = np.median(rotated_array[:, 2])

      # Calculate noise characteristics
      z_std = np.std(rotated_array[:, 2])
      self.R = np.array([[z_std ** 2]])  # Update measurement noise

      print(f"Calibration complete:")
      print(f"Rotation angle: {np.degrees(self.rotation_angle):.1f} degrees")
      print(f"Baselines - X: {self.baseline_x:.1f}, Y: {self.baseline_y:.1f}, Z: {self.baseline_z:.1f}")
      print(f"Z standard deviation: {z_std:.2f}")


    def predict(self):
        """Predict step of the Kalman filter"""
        self.state = self.F @ self.state
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, x: float, y: float, z: float):
        """Update step of the Kalman filter"""
        if self.baseline_z is None:
            raise ValueError("Must calibrate detector first!")

        # Calculate vertical acceleration (removing baseline)
        accel = z - self.baseline_z

        # Measurement matrix (we observe acceleration)
        H = np.array([[0, 1]])

        # Innovation
        y = np.array([accel * self.dt]) - H @ self.state

        # Innovation covariance
        S = H @ self.P @ H.T + self.R

        # Kalman gain
        K = self.P @ H.T @ inv(S)

        # Update state
        self.state = self.state + K @ y

        # Update covariance
        self.P = (np.eye(2) - K @ H) @ self.P

    def detect_jump(self, x: float, y: float, z: float) -> Tuple[bool, str, float]:
        """
        Process new measurements and detect jumps
        Returns: (event_detected, event_type, velocity)
        """
        self.predict()
        x_rod, y_rod, z_rod = rotate_coordinates(x, y, z, self.rotation_angle)
        self.samples.append((x_rod, y_rod, z_rod))
        self.update(x_rod, y_rod, z_rod)
        z_accel = z_rod - self.baseline_z
        print(f"Z_accel value is {z_accel}")
        # Jump start detection
        if not self.in_jump and z_accel > self.jump_threshold:
            self.in_jump = True
            return True, "jump_start", self.state[1]

        # Landing detection
        elif self.in_jump and z_accel < self.landing_threshold:
            self.in_jump = False
            return True, "landing", self.state[1]

        return False, "none", self.state[1]

    # def process_and_plot(self, samples: List[Tuple[float, float, float]]):
    def process_and_plot(self):
      """
      Process samples through detector and create debug plots

      Args:
          detector: Initialized and calibrated JumpDetector instance
          samples: List of (x, y, z) tuples to process
      """
      # Storage for plotting
      plot_data = []

      # Process all samples
      for x, y, z in self.samples:
          # Store current state before detection
          z_accel = z - self.baseline_z
          plot_data.append({
              'z_acceleration': z_accel,
              'position': self.state[0],
              'velocity': self.state[1],
              'in_jump': self.in_jump,
              'raw_z': z,
              'raw_x': x,
              'raw_y': y,
          })

          # Run detection
          event_detected, event_type, velocity = self.detect_jump(x, y, z)

      # Extract time series data
      times = range(len(plot_data))
      z_accels = [d['z_acceleration'] for d in plot_data]
      velocities = [d['velocity'] for d in plot_data]
      positions = [d['position'] for d in plot_data]
      in_jump = [d['in_jump'] for d in plot_data]
      raw_z = [d['raw_z'] for d in plot_data]
      raw_x = [d['raw_x'] for d in plot_data]


      # Create plots
      fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(12, 10), sharex=True)

      # Plot raw Z values
      ax1.plot(times, raw_x, label='Raw X')
      ax1.plot(times, raw_z, label='Raw Z')
      ax1.axhline(self.baseline_z, color='k', linestyle='--', label='Baseline')
      ax1.set_ylabel('Raw Z')
      ax1.legend()

      # Plot acceleration and thresholds
      ax2.plot(times, z_accels, label='Z Acceleration')
      ax2.axhline(self.jump_threshold, color='g', linestyle='--', label='Jump Threshold')
      ax2.axhline(self.landing_threshold, color='r', linestyle='--', label='Landing Threshold')
      ax2.set_ylabel('Acceleration')
      ax2.legend()

      # Plot velocity
      ax3.plot(times, velocities, label='Estimated Velocity')
      ax3.set_ylabel('Velocity')
      ax3.legend()

      # Plot position and jump state
      ax4.plot(times, positions, label='Estimated Position')
      ax4.fill_between(times, min(positions), max(positions), where=in_jump,
                      alpha=0.3, color='g', label='In Jump')
      ax4.set_ylabel('Position')
      ax4.set_xlabel('Time Steps')
      ax4.legend()

      plt.tight_layout()
      plt.show()

base_url = "https://gist.githubusercontent.com/eloycoto/e075714ff108513e44c8bc05a57394dc/raw/1ddf1e9fdafdd6b649471eebc94e967a8d305097/"

import requests

def parse_data(name):
  # Parse the data

  req = requests.get(f"{base_url}/{name}")
  if req.status_code != 200:
    raise Exception("Invalid file to parse")
  data = []
  for line in req.text.strip().split('\n'):
      if line.strip():
          x, y, z, _ = map(float, line.strip().split(','))

          # Convert raw values to acceleration (m/s²)
          # Using a smaller sensitivity factor to get more realistic values
          SCALE_FACTOR = 1024.0  # Increased denominator to reduce the scale
          x_ms2 = (x / SCALE_FACTOR) * 9.81
          y_ms2 = (y / SCALE_FACTOR) * 9.81
          z_ms2 = ((z - 256) / SCALE_FACTOR) * 9.81

          data.append((x_ms2, y_ms2, z_ms2))
  return data


values_to_text = [
    #"four_jumps_z_axis_correct.txt",
    #"two_jumps_z_axis_semmi_correct.txt",
    #"four_jumps_x_axis.txt",
    #"three_jumps_in_z_noise.txt",
    "five_jumps_in_x_stable.txt"
]
#values_to_text = ["four_jumps_z_axis_correct.txt","five_jumps_in_x_stable.txt"]
for x in values_to_text:
    print("-------------------------------------------------------")
    print(f"# Parsing data ***{x}***")
    data = parse_data(x)

    # Initialize detector with calibration samples
    detector = JumpDetector(data[:10])
    samples = []

    for i, (x, y, z) in enumerate(data):
        samples.append((x, y, z))
        event_detected, event_type, velocity = detector.detect_jump(x, y, z)

        if event_detected:
            # Get corrected reading
            _, _, corrected_z = detector.detect_jump(x, y, z)
            z_accel = corrected_z + 1.0  # Shift from -1 to 0 baseline

            print(f"Time: {i*0.1:4.1f}s  Event: {event_type:<10}  "
                  f"Velocity: {velocity:6.2f} m/s  Accel: {z_accel:6.2f} g")
