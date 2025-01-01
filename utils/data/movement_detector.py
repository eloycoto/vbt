import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt
from typing import List, Tuple
import sys

def calculate_rotation(samples: List[Tuple[float, float, float]]) -> float:
    """
    Calculate rotation angle from calibration samples by analyzing the x-y plane.
    Assumes the sensor should ideally be aligned with the Y axis when standing still.
    """
    samples_array = np.array(samples)
    mean_x = np.mean(samples_array[:, 0])
    mean_y = np.mean(samples_array[:, 1])
    rotation_angle = np.arctan2(mean_x, mean_y)
    return rotation_angle

def rotate_coordinates(x: float, y: float, z: float, angle: float) -> Tuple[float, float, float]:
    """
    Rotate coordinates around the Z axis by given angle.
    """
    cos_angle = np.cos(angle)
    sin_angle = np.sin(angle)
    rotation_matrix = np.array([
        [cos_angle, -sin_angle, 0],
        [sin_angle, cos_angle, 0],
        [0, 0, 1]
    ])
    rotated_coords = rotation_matrix @ np.array([x, y, z])
    return tuple(rotated_coords)

class MovementDetector:

    def __init__(self, calibration_samples: List[Tuple[float, float, float]] = None):
        self.samples = []
        self.events = []
        self.plot_data = []
        self.dt = 0.1

        self.state = np.zeros(2)
        self.F = np.array([[1, self.dt], [0, 1]])
        self.Q = np.array([[0.5, 0], [0, 1.0]])
        self.R = np.array([[25.0]])
        self.P = np.array([[1, 0], [0, 1]])

        # Will be set during calibration
        self.baseline_z = None
        self.baseline_x = None
        self.baseline_y = None
        self.rotation_angle = None

        # Movement detection parameters
        self.up_threshold = 0.3      # Reduced threshold for upward movement
        self.down_threshold = -0.3   # Threshold for downward movement
        self.in_movement = False
        self.movement_type = None    # Can be 'down', 'up', or None
        self.position_baseline = 0
        self.min_movement_duration = 0.5  # Minimum duration (seconds) for valid movement

        # Movement state tracking
        self.movement_start_time = None
        self.last_position = 0
        self.in_squat = False

        if calibration_samples:
            self.calibrate(calibration_samples)

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

    def calibrate(self, samples: List[Tuple[float, float, float]]):
        """Calculate baseline values and rotation from static samples"""
        # Calculate rotation angle
        rotated_samples = []
        total_xy = 0
        total_z = 0

        for x, y, z in samples:
            total_xy += np.sqrt(x*x + y*y)
            total_z += abs(z)

        # Calculate rotation angle based on average ratio
        self.rotation_angle = np.arctan2(total_xy/len(samples), total_z/len(samples))

        # Rotate all samples and calculate baselines
        for x, y, z in samples:
            x_rot, y_rot, z_rot = rotate_coordinates(x, y, z, self.rotation_angle)
            rotated_samples.append((x_rot, y_rot, z_rot))

        rotated_array = np.array(rotated_samples)

        # Calculate baselines using median to be robust against outliers
        self.baseline_x = np.median(rotated_array[:, 0])
        self.baseline_y = np.median(rotated_array[:, 1])
        self.baseline_z = np.median(rotated_array[:, 2])

        # Calculate noise characteristics
        z_std = np.std(rotated_array[:, 2])
        self.R = np.array([[z_std ** 2]])  # Update measurement noise

        # Adjust detection thresholds based on noise level
        base_threshold = max(0.2, z_std * 2)  # Minimum threshold of 0.2g
        self.up_threshold = base_threshold
        self.down_threshold = -base_threshold

        print(f"Calibration complete:")
        print(f"Rotation angle: {np.degrees(self.rotation_angle):.1f} degrees")
        print(f"Baselines - X: {self.baseline_x:.3f}, Y: {self.baseline_y:.3f}, Z: {self.baseline_z:.3f}")
        print(f"Z standard deviation: {z_std:.3f}")
        print(f"Adjusted thresholds - Up: {self.up_threshold:.3f}g, Down: {self.down_threshold:.3f}g")



    def _process(self, x: float, y: float, z: float) -> Tuple[bool, str, float]:

        self.predict()
        x_rot, y_rot, z_rot = rotate_coordinates(x, y, z, self.rotation_angle)
        z_accel = z_rot - self.baseline_z

        current_time = len(self.plot_data) * self.dt
        plot_data = {
            'time': current_time,
            'z_acceleration': z_accel,
            'position': self.state[0],
            'velocity': self.state[1],
            'in_movement': self.in_movement,
            'movement_type': self.movement_type,
            'event': "",
            'raw_z': z_rot,
            'raw_x': x_rot,
            'raw_y': y_rot
        }

        # Store data for plotting
        self.plot_data.append(plot_data)
        self.update(x_rot, y_rot, z_rot)
        return plot_data

    def process_sample(self, x: float, y: float, z: float, detect_movements: bool = True) -> Tuple[bool, str, float]:

        data = self._process(x,y,z)

        # print(f'Z: {data["raw_z"]:.3f}, accel: {data["z_acceleration"]:.3f}')

        current_idx = len(self.plot_data) - 1

        # Get current data point
        current_time = len(self.plot_data) - 1
        current_accel = self.plot_data[current_time]['z_acceleration']
        current_velocity = self.plot_data[current_time]['velocity']

        # Initialize tracking
        if not hasattr(self, 'lowest_point'):
            self.lowest_point = {'accel': current_accel, 'time': current_time, 'z': z}
            self.in_upward_movement = False
            self.movement_start = False
            self.peak_found = False
            self.peak_point = {'time': 0, "acceleration": 0}



        # Track lowest point
        # if data["raw_z"] <= -1.790 and data["raw_z"] >= -1.792:
        #     import ipdb;ipdb.set_trace()

        # if data["z_acceleration"] > 1.20:
        #     import ipdb; ipdb.set_trace()

        if current_accel < self.lowest_point['accel']:
            self.lowest_point = {'accel': current_accel, 'time': current_time, 'z': z}
            self.movement_start = True

        # Detect significant upward movement
        if current_accel > 0.2:  # Threshold for movement start
            self.in_upward_movement = True
            self.movement_start = False

        is_peak = lambda : (self.peak_found and current_accel > self.peak_point["acceleration"]) or (not self.peak_found and current_accel > 0)

        # if is_peak():
        #     print("this can be updated", current_time)

        # Find peak and calculate velocity
        #if self.in_upward_movement and not self.peak_found and current_accel > 0.8:  # Peak threshold
        if self.in_upward_movement and is_peak():  # Peak threshold
            time_diff = (current_time - self.lowest_point['time']) * self.dt
            lowest_time = self.lowest_point["time"]
            if current_time - lowest_time > 3:
                z_diff = z - self.lowest_point['z']
                velocity = z_diff / time_diff if time_diff > 0 else 0
                print(f"\nMovement detected at point {current_time}:")
                print(f"Start Z: {self.lowest_point['z']:.3f}, End Z: {z:.3f}")
                print(f"Time difference: {time_diff:.3f} seconds")
                print(f"Calculated velocity: {velocity:.3f} m/s")
                self.plot_data[current_time]['peak_velocity'] = velocity
                self.plot_data[self.lowest_point.get("time")]["event"] = "lower"
                self.peak_found = True
                self.peak_point["time"] = current_time
                self.peak_point["acceleration"] = current_accel
                return True, "up", velocity


        # if current_time > 142:
        #     import ipdb; ipdb.set_trace()

        # Reset detection after peak
        # @TODO this current_accel, should be reset when the peak was found?
        # What happens if the accell was all negative points?
        if self.peak_found and current_accel < -0.1:  # Reset threshold
            # Just to make sure that when it's peak, there is no a movement to
            # get down.
            if (current_time - self.peak_point["time"]) > 4:
                self.plot_data[self.peak_point["time"]]["event"] = "peak"
                self.lowest_point = {'accel': current_accel, 'time': current_time, 'z': z}
                self.in_upward_movement = False
                self.movement_start = False
                self.peak_found = False

        return False, "none", 0



    def plot_raw_data(self, name="Raw data analysis"):
        times = [d['time'] for d in self.plot_data]
        raw_z = [d['raw_z'] for d in self.plot_data]
        raw_x = [d['raw_x'] for d in self.plot_data]
        raw_y = [d['raw_y'] for d in self.plot_data]
        events = [d['event'] for d in self.plot_data]

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 12), sharex=True)
        fig.suptitle(name, fontsize=14)

        # Plot 1: Raw data
        ax1.plot(times, raw_z, label='Z', color='green')
        ax1.plot(times, raw_x, label='X', color='red')
        ax1.set_ylabel('Raw data with rotation (g)')
        ax1.legend(loc='upper right')
        ax1.grid(True, alpha=0.3)

        z_accels = [d['z_acceleration'] for d in self.plot_data]
        ax2.plot(times, z_accels, label='Vertical Acceleration points', color='black')
        ax2.scatter(times, z_accels, label='Dots', color='red')

        color_map = {
            # 'up': 'green',
            'peak': 'green',
            'lower': 'blue',
        }
        for i in range(len(times)-1):
            color = color_map.get(events[i], "white")
            ax2.axvspan(
                times[i],
                times[i+1],
                alpha=0.2,
                label=events[i],
                color=color)

        handles, labels = ax2.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))

        current_ticks = list(ax2.get_xticks())  # Get current ticks
        all_ticks = sorted(list(set(current_ticks + [times[i] for i in range(len(events)) if events[i] == 'peak'])))
        ax2.set_xticks(all_ticks)

        ax2.legend(by_label.values(), by_label.keys(), loc='upper right')

        plt.tight_layout()
        plt.savefig("plot.svg")

    def plot(self):
        """Generate visualization plots"""
        if not self.plot_data:
            print("No data to plot!")
            return

        # Extract time series data
        times = [d['time'] for d in self.plot_data]
        z_accels = [d['z_acceleration'] for d in self.plot_data]
        velocities = [d['velocity'] for d in self.plot_data]
        positions = [d['position'] for d in self.plot_data]
        movement_types = [d['movement_type'] for d in self.plot_data]

        # Create plots
        # fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(15, 12), sharex=True)
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 12), sharex=True)
        fig.suptitle('Movement Detection Analysis', fontsize=14)

        # Plot 1: Vertical acceleration and thresholds
        ax1.plot(times, z_accels, label='Vertical Acceleration points', color='yellow')
        ax1.scatter(times, z_accels, label='Vertical Acceleration', color='blue')
        ax1.scatter(times, positions, label='positions', color='pink')
        ax1.scatter(times, velocities, label='Vertical velocity', color='black')
        ax1.axhline(self.up_threshold, color='g', linestyle='--',
                    label=f'Up Threshold ({self.up_threshold:.2f}g)')
        ax1.axhline(self.down_threshold, color='r', linestyle='--',
                    label=f'Down Threshold ({self.down_threshold:.2f}g)')
        ax1.set_ylabel('Acceleration (g)')
        ax1.legend(loc='upper right')
        ax1.grid(True, alpha=0.3)
        ax1.set_title('Vertical Acceleration and Detection Thresholds')

        # Plot 2: Estimated velocity
        # ax2.plot(times, velocities, label='Vertical Velocity', color='purple')
        # ax2.set_ylabel('Velocity (m/s)')
        ax2.plot(times, z_accels, label='Vertical accels', color='purple')
        ax2.plot(times, z_accels, label='Vertical accelerations', color='purple')
        color_map = {
            'waiting': 'gray',
            'takeoff': 'green',
            'flight': 'red',
            'landing': 'orange',
            'jump_up': 'blue',
            'jump_down': 'magenta'
        }

        for i in range(len(times)-1):
            current_type = movement_types[i]
            if current_type in color_map:
                ax2.axvspan(times[i], times[i+1],
                           alpha=0.2,
                           color=color_map[current_type],
                           label=current_type if current_type not in [t[0] for t in ax2.get_legend_handles_labels()] else "")

        ax2.set_xlabel('Time (s)')
        ax2.grid(True, alpha=0.3)
        ax2.set_title('Velocity and Movement Phases')

        # Create legend without duplicate labels
        handles, labels = ax2.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax2.legend(by_label.values(), by_label.keys(), loc='upper right')

        plt.tight_layout()
        plt.savefig("plot.svg")
        # plt.show()
        # import ipdb; ipdb.set_trace()

        # Print summary statistics

base_url = "https://gist.githubusercontent.com/eloycoto/e075714ff108513e44c8bc05a57394dc/raw/4f448041908231ab533cd4c5bd0572f06099413b/"

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
    "four_jumps_z_axis_correct.txt",
    # "two_jumps_z_axis_semmi_correct.txt",
    # "four_jumps_x_axis.txt",
    # "three_jumps_in_z_noise.txt",
    "five_jumps_in_x_stable.txt",
    "five_kid_squats_in_z.txt",
    "five_kid_rdl_z.txt"
]
#values_to_text = ["four_jumps_z_axis_correct.txt","five_jumps_in_x_stable.txt"]
for name in values_to_text:
    print("-------------------------------------------------------")
    print(f"# Parsing data ***{name}***")
    data = parse_data(name)

    # Initialize detector with calibration samples
    detector = MovementDetector(data[:10])
    samples = []
    for i, (x, y, z) in enumerate(data[10:]):  # Skip calibration samples
            event_detected, event_type, velocity = detector.process_sample(x, y, z)

            if event_detected:
                # Get rotated coordinates
                x_rot, y_rot, z_rot = rotate_coordinates(x, y, z, detector.rotation_angle)
                z_accel = z_rot - detector.baseline_z

                print(f"Time: {i*detector.dt:4.1f}s  Event: {event_type:<10}  "
                      f"Velocity: {velocity:6.2f} m/s  Accel: {z_accel:6.2f} g")

    # Generate plots
    # detector.plot()
    # for data in detector.plot_data:
    #     print(f'For Z {data["raw_z"]:.3f}: \tAccel: {data["z_acceleration"]:.3f}, \tvelocity: {data["velocity"]:.3f}')

    detector.plot_raw_data(name)
    sys.stdin.read(1)
    print("\n")
