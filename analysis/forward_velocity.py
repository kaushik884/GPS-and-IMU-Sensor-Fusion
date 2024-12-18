import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import integrate
import math
from scipy.signal import butter, filtfilt


# Load IMU and GPS CSV files
imu_data = pd.read_csv('imu_data_driving.csv')
gps_data = pd.read_csv('gps_data_driving.csv')

# Combine seconds and nanoseconds into a single time column for both IMU and GPS
imu_data['time'] = imu_data['time_sec'] + imu_data['time_nanosec'] / 1e9
gps_data['time'] = gps_data['time_sec'] + gps_data['time_nanosec'] / 1e9

# Calculate IMU-based velocity by integrating Linear.x (forward acceleration) over time
linear_acc_x = imu_data['la_x'].to_numpy()
imu_time = imu_data['time_sec'].to_numpy()

# Remove mean acceleration to correct for any static offset
linear_acc_x -= np.mean(linear_acc_x)

# Integrate forward acceleration to get forward velocity (IMU-based velocity)
forward_velocity_imu = integrate.cumulative_trapezoid(linear_acc_x, imu_time, initial=0)

# Calculate GPS-based velocity from UTM coordinates
utm_easting = gps_data['utm_easting'].to_numpy()
utm_northing = gps_data['utm_northing'].to_numpy()
gps_time = gps_data['time_sec'].to_numpy()

# Compute distance between consecutive UTM coordinates
distance = np.sqrt(np.diff(utm_easting)**2 + np.diff(utm_northing)**2)
gps_velocity = distance / np.diff(gps_time)

# Adjust GPS time to match GPS velocity array length
gps_time = gps_time[1:]

# Plot both velocity estimates (Original IMU-based and GPS-based velocities)
plt.figure(figsize=(12, 6))
plt.plot(imu_time, forward_velocity_imu, label='IMU-based Velocity (Original)', color='blue')
plt.plot(gps_time, gps_velocity, label='GPS-based Velocity', color='red')
plt.title('Forward Velocity from IMU and GPS')
plt.xlabel('Time (seconds)')
plt.ylabel('Velocity (m/s)')
plt.legend()
plt.grid(True)
plt.show()

def low_pass_filter(data, cutoff_freq, fs, order=4):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff_freq / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, data)

# Sampling rate (assuming time steps are consistent for simplicity)
imu_sampling_rate = 1 / np.mean(np.diff(imu_time))  # Hz

# Apply a low-pass filter to forward acceleration data to remove high-frequency noise
cutoff_frequency = 1.0  # Cutoff frequency in Hz (tuneable parameter)
linear_acc_x_filtered = low_pass_filter(linear_acc_x, cutoff_frequency, imu_sampling_rate)

# Re-integrate the filtered acceleration to obtain a smoother IMU-based velocity
forward_velocity_imu_corrected = integrate.cumulative_trapezoid(linear_acc_x_filtered, imu_time, initial=0)

# Apply adjustments for the second plot (Zero out negative velocities only)
# Zero out any negative velocities in IMU-based velocity
adjusted_velocity_imu = np.copy(forward_velocity_imu_corrected)
adjusted_velocity_imu[adjusted_velocity_imu < 0] = 0

# Plot the corrected IMU velocity with GPS velocity (Adjusted Plot without filter)
plt.figure(figsize=(12, 6))
plt.plot(imu_time, adjusted_velocity_imu, label='Corrected IMU-based Velocity (No Filter)', color='blue')
plt.plot(gps_time, gps_velocity, label='GPS-based Velocity', color='red')
plt.title('Corrected IMU and GPS Velocity Comparison')
plt.xlabel('Time (seconds)')
plt.ylabel('Velocity (m/s)')
plt.legend()
plt.grid(True)
plt.show()


# Now, integrate to calculate displacement in the X direction
displacement_x_imu = integrate.cumulative_trapezoid(adjusted_velocity_imu, initial=0)
int_gps_vel = integrate.cumulative_trapezoid(distance, initial=0)

acc_x = imu_data['la_x']
time_imu = imu_data['time_sec'] + imu_data['time_nanosec']*1e-9
xdot = acc_x
x1dot = integrate.cumulative_trapezoid(xdot)

ang_z = imu_data['av_z']
y2dot = ang_z[1:] * x1dot

# Step 2.2: Compare with lateral acceleration ÿ_obs (IMU's `la_y`)
y_obs = imu_data['la_y']

plt.figure(figsize=(16, 8))
plt.plot(y_obs, label='Y observed', color='green')
plt.plot(y2dot/1000, label='wX(dot)', color='purple')
plt.legend(loc='upper right', fontsize='x-large')
plt.grid(color='grey', linestyle='--', linewidth=1)
plt.title('Comparison of $\omega Ẋ$ with $ÿ_{obs}$')
plt.xlabel('Time (secs)')
plt.ylabel('Acceleration (m/s²)')
plt.show()