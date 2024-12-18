import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import integrate
from scipy.optimize import leastsq
from scipy.signal import butter
from scipy import signal

# Load the uploaded data file
data_path = 'imu_data_driving.csv'
data = pd.read_csv(data_path)

# Convert magnetic x and y data to numpy arrays
magnetic_x = data['mag_x'].to_numpy()
magnetic_y = data['mag_y'].to_numpy()

# Hard-iron calibration
x_offset = (magnetic_x.min() + magnetic_x.max()) / 2.0
y_offset = (magnetic_y.min() + magnetic_y.max()) / 2.0
calibrated_x = magnetic_x - x_offset
calibrated_y = magnetic_y - y_offset

# Define ellipse fit function for soft-iron calibration
def ellipse_fit(params, x, y):
    x0, y0, a, b, theta = params
    cos_theta, sin_theta = np.cos(theta), np.sin(theta)
    x_trans = cos_theta * (x - x0) + sin_theta * (y - y0)
    y_trans = -sin_theta * (x - x0) + cos_theta * (y - y0)
    return (x_trans / a) ** 2 + (y_trans / b) ** 2 - 1

# Initial guess for ellipse parameters
initial_guess = [0, 0, np.std(calibrated_x), np.std(calibrated_y), 0]

# Perform least-squares ellipse fitting
params, _ = leastsq(ellipse_fit, initial_guess, args=(calibrated_x, calibrated_y))
x0, y0, a, b, theta = params

# Construct rotation matrix
cos_theta, sin_theta = np.cos(theta), np.sin(theta)
R = np.array([[cos_theta, -sin_theta], [sin_theta, cos_theta]])

# Apply the soft-iron calibration without scaling and normalize
soft_iron_calibrated = np.dot(R, np.array([calibrated_x - x0, calibrated_y - y0]))

# Normalize to match the original radius
raw_radius_mean = np.mean(np.sqrt(magnetic_x**2 + magnetic_y**2))
calibrated_radius_mean = np.mean(np.sqrt(soft_iron_calibrated[0]**2 + soft_iron_calibrated[1]**2))
scaling_factor = raw_radius_mean / calibrated_radius_mean

# Apply normalization scaling factor
final_calibrated_x, final_calibrated_y = scaling_factor * soft_iron_calibrated

# Calculate mean radius for hard-iron and soft-iron calibrated data
hard_iron_radius_mean = np.mean(np.sqrt(calibrated_x**2 + calibrated_y**2))
soft_iron_radius_mean = np.mean(np.sqrt(final_calibrated_x**2 + final_calibrated_y**2))

# Compute scaling factor to match the radii
normalization_scaling_factor = hard_iron_radius_mean / soft_iron_radius_mean

# Apply scaling factor to soft-iron calibrated data
normalized_final_calibrated_x = final_calibrated_x * normalization_scaling_factor
normalized_final_calibrated_y = final_calibrated_y * normalization_scaling_factor
# Calculate raw and corrected yaw angles (in degrees)
raw_yaw = np.degrees(np.arctan2(magnetic_y, magnetic_x))
corrected_yaw = np.degrees(np.arctan2(normalized_final_calibrated_x, normalized_final_calibrated_y))

# Plot raw vs corrected yaw angles
plt.figure(figsize=(14, 7))
plt.plot(np.unwrap(raw_yaw), label="Raw Yaw Angle")
plt.plot(np.unwrap(corrected_yaw-100), label="Corrected Yaw Angle")
plt.xlabel("Sample Index")
plt.ylabel("Yaw Angle (degrees)")
plt.title("Comparison of Raw and Corrected Yaw Angles")
plt.legend()
plt.grid(True)
plt.show()


# Plot comparison of yaw angles from magnetometer and integrated gyro yaw
gyro_int = integrate.cumulative_trapezoid(data['av_z'], initial=0)
plt.figure(figsize=(14, 7))
plt.plot(np.unwrap(gyro_int), label='Gyro Integrated Yaw', c='palevioletred')
plt.plot(np.unwrap(corrected_yaw-100), label="Corrected Yaw Angle")
plt.xlabel("Sample Index")
plt.ylabel("Yaw Angle (degrees)")
plt.title("Comparison of Yaw Angles: Magnetometer vs. Integrated Gyro")
plt.legend()
plt.grid(True)
plt.show()

lpf = signal.filtfilt(*butter(3, 0.1, "lowpass",fs = 40, analog=False), np.unwrap(corrected_yaw-100))
hpf = signal.filtfilt(*butter(3, 0.0001, 'highpass', fs = 40, analog=False), np.unwrap(gyro_int))

plt.figure(figsize = (16,8))
plt.plot(lpf, label='LPF Calibrated Yaw')
plt.legend(loc='upper right', fontsize='x-large')
plt.grid(color='grey', linestyle='--', linewidth=1)
plt.plot(hpf, label = 'HPF Gyro Yaw', c='seagreen')
plt.legend(loc='upper right', fontsize='x-large')
plt.grid(color='grey', linestyle='--', linewidth=1)
plt.title('LPF for Magnetic Yaw and HPF for Gyro Yaw')
plt.xlabel('Samples @ 40Hz')
plt.ylabel('Yaw (degrees)')
plt.show()



w, x, y, z = data['q_w'], data['q_x'], data['q_y'], data['q_z']

t3 = +2.0 * (w * z + x * y)
t4 = +1.0 - 2.0 * (y * y + z * z)
yaw_imu = np.degrees(np.unwrap(np.arctan2(t3, t4)))
#Original Yaw V/S Calibrated Yaw
alpha = 0.75
omega = data['av_z']
yaw_filtered = []
yaw_filtered = np.append(yaw_filtered,0)
for i in range(len(omega) - 1):
  j = i+1
  yaw_filtered = np.append(yaw_filtered, alpha*(yaw_filtered[i] + hpf[j]*0.05) + ((1-alpha)*lpf[j]))
# lpf1 = 1 - hpf1
# yaw_filtered = (hpf1*hpf) + (lpf1*lpf)
plt.figure(figsize=(16, 8))
plt.plot(yaw_filtered, label='Complementary Filter')
plt.plot(yaw_imu, label='Yaw computed by IMU')
plt.legend(loc='lower right', fontsize='x-large')
plt.grid(color='grey', linestyle='--', linewidth=1)
plt.xlabel('Samples @ 40 Hz')
plt.ylabel('Yaw (degrees)')
plt.title('IMU Yaw vs Complementary Filter Yaw')
plt.show()

plt.figure(figsize = (16,8))
plt.plot(lpf, label='LPF Calibrated Yaw',c= 'teal')
plt.legend(loc = "upper right")
plt.plot(hpf, label = 'HPF Gyro Yaw')
plt.plot(yaw_filtered, label='Complementary Filter',c= 'crimson')
plt.plot(yaw_imu, label='Yaw computed by IMU')
plt.legend(loc='upper right', fontsize='x-large')
plt.grid(color='grey', linestyle='--', linewidth=1)
plt.legend(loc="upper right")
plt.xlabel('Samples @ 40Hz')
plt.ylabel('Yaw (degrees)')
plt.title('LPF for Magnetic Yaw V/S HPF for Gyro Yaw V/S  Complimentary Yaw')
plt.show()