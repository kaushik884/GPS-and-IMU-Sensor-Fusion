import math
import statistics
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.integrate import cumulative_trapezoid
from scipy.signal import butter, filtfilt

# Read data
imu_data = pd.read_csv('imu_data_driving.csv')
gps_data = pd.read_csv('gps_data_driving.csv')

# Load data
time1 = imu_data['time_sec']
time2 = gps_data['time_sec']
mag_z = imu_data['mag_z']
mag_x = imu_data['mag_x']
mag_y = imu_data['mag_y']
gyro_z = imu_data['av_z']
accel_x = imu_data['la_x']
accel_y = imu_data['la_y']
gps_easting = gps_data['utm_easting']
gps_northing = gps_data['utm_northing']
w, x, y, z = imu_data['q_w'], imu_data['q_x'], imu_data['q_y'], imu_data['q_z']

t3 = +2.0 * (w * z + x * y)
t4 = +1.0 - 2.0 * (y * y + z * z)
yaw_i = np.degrees(np.unwrap(np.arctan2(t3, t4)))

# Normalize time
time1 = time1 - time1[0]
time2 = time2 - time2[0]

# Unwrap yaw
yaw_imu = np.unwrap(yaw_i)

# Hard iron offset
offset_x = (mag_x.max() + mag_x.min()) / 2
offset_y = (mag_y.max() + mag_y.min()) / 2
print(f'hard iron offset x = {offset_x}, y = {offset_y}')
mag_x_h = mag_x - offset_x
mag_y_h = mag_y - offset_y

# Soft iron calibration
radius_x = (mag_x_h.max() - mag_x_h.min()) / 2  # radius x
radius_y = (mag_y_h.max() - mag_y_h.min()) / 2  # radius y
radius = math.sqrt(radius_x**2 + radius_y**2)
theta = math.atan2(radius_y, radius_x)  # theta

# Rotation
rot_mat = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])
rot_data = np.matmul(rot_mat, np.vstack((mag_x_h, mag_y_h)))

# Scaling
des_r = 0.2
scale_x = des_r / radius_x
scale_y = des_r / radius_y
scale_mat = np.array([[scale_x, 0], [0, scale_y]])
scale_data = np.matmul(scale_mat, rot_data)

# Reverse rotation
rev_rot_mat = np.array([[np.cos(-theta), np.sin(-theta)], [-np.sin(-theta), np.cos(-theta)]])
final_data = np.matmul(rev_rot_mat, scale_data)

# Yaw calculation
yaw = np.arctan2(mag_y, mag_x)
yaw_deg = np.degrees(np.unwrap(yaw))
yaw_calibrated = np.arctan2(final_data[0], final_data[1])
yaw_calibrated_deg = np.degrees(np.unwrap(yaw_calibrated))
yaw_angle = cumulative_trapezoid(gyro_z, time1, initial=0)
yaw_angle_deg = np.degrees(np.unwrap(yaw_angle))

# LPF, HPF, complementary filter
alpha = 0.9
def lpf(data, cutoff, fs, order=2):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, data)

def hpf(data, cutoff, fs, order=2):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    return filtfilt(b, a, data)

fs = 1 / np.mean(np.diff(time1))
fs1 = 1 / np.mean(np.diff(time1))
cutoff_lpf = 0.1
yaw_calibrated_deg_f = lpf(yaw_calibrated_deg, cutoff=cutoff_lpf, fs=fs1)
cutoff_hpf = 0.0001
yaw_angle_deg_f = hpf(yaw_angle_deg, cutoff=cutoff_hpf, fs=fs)
yaw_complementary = alpha * yaw_angle_deg + (1 - alpha) * yaw_calibrated_deg

# Velocity and displacement calculations
vel_ac = cumulative_trapezoid(accel_x, time1, initial=0)
vel_gps = np.sqrt(np.diff(gps_easting)**2 + np.diff(gps_northing)**2) / np.diff(time2)
vel_gps = np.insert(vel_gps, 0, 0)
ac_c = accel_x - np.mean(accel_x)
vel_ac_c = cumulative_trapezoid(ac_c, time1, initial=0)
vel_ac_c[vel_ac_c < 0] = 0
mask = (vel_gps > 0) & (vel_gps < 0.18)
vel_ac_c[np.isin(time1, time2[mask])] = 0

# Dead reckoning and displacement
disp = cumulative_trapezoid(vel_ac_c, initial=0)
x2dot = accel_x
x1dot = cumulative_trapezoid(accel_x, time1, initial=0)
y2dot = gyro_z * x1dot
Y_obs = accel_y
fv = vel_ac_c
yc = yaw_complementary
ve = fv * np.sin(np.radians(yaw_imu - 19))
vn = fv * np.cos(np.radians(yaw_imu - 19))
xe = cumulative_trapezoid(ve, time1, initial=0)
xn = cumulative_trapezoid(vn, time1, initial=0)

# Scaling and alignment for plots
scaling = 0.70
xe_s = xe * scaling
xn_s = xn * scaling
xe_a = xe_s - xe_s[0] + gps_easting[0]
xn_a = xn_s - xn_s[0] + gps_northing[0]

larger_correction_angle = np.radians(6)  # Larger rotation correction angle

# Rotation matrix for larger correction
larger_correction_matrix = np.array([[np.cos(larger_correction_angle), -np.sin(larger_correction_angle)],
                                     [np.sin(larger_correction_angle), np.cos(larger_correction_angle)]])

# Apply larger rotation correction to the estimated trajectory
corrected_trajectory_larger = np.matmul(larger_correction_matrix, np.vstack((xe_s, xn_s)))

# Adjust the corrected trajectory to align starting points with GPS
xe_a_corrected_larger = corrected_trajectory_larger[0] - corrected_trajectory_larger[0][0] + gps_easting[0]
xn_a_corrected_larger = corrected_trajectory_larger[1] - corrected_trajectory_larger[1][0] + gps_northing[0]


# Plotting
plt.figure(figsize=(10, 6))
plt.plot(gps_easting, gps_northing, label='GPS track', color='blue')
plt.plot(xe_a_corrected_larger, xn_a_corrected_larger, label='Estimated track', color='red')
plt.xlabel('Eastward Position (m)')
plt.ylabel('Northward Position (m)')
plt.title('Comparison of GPS and Estimated Trajectory')
plt.legend()
plt.grid(True)

plt.figure(figsize=(10, 6))
plt.plot(xe_a, xn_a, label='Estimated track', color='blue')
plt.xlabel('xe (m)')
plt.ylabel('xn (m)')
plt.title('Estimated Trajectory')
plt.legend()
plt.grid(True)

plt.figure(figsize=(10, 10))
plt.plot(Y_obs, label='Y Observed', color='steelblue')
plt.plot(y2dot / -1, label='wX(dot)', color='orangered')
plt.legend(loc='upper right', fontsize='x-large')
plt.grid(color='grey', linestyle='--', linewidth=1)
plt.title('Y Observed vs wX(dot)')
plt.xlabel('Samples @40Hz')
plt.ylabel('Acceleration (m/s²)')
plt.show()
