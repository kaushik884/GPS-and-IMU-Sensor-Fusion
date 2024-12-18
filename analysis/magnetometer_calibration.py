import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import leastsq

# Load the CSV file
file_path = 'imu_data_circles.csv'  # Replace with your file path
data = pd.read_csv(file_path)

# Combine time_sec and time_nanosec into a single time column
data['time'] = data['time_sec'] + data['time_nanosec'] / 1e9

# Ignore the first 30 time_sec and last 30 time_sec
start_time = 1729525384
end_time = 1729525422
data = data[(data['time_sec'] > start_time) & (data['time_sec'] < end_time)]

# Extract magnetometer data and convert to numpy arrays
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

plt.figure(figsize=(24, 6))
# Raw vs Hard-Iron Calibrated
plt.subplot(1, 2, 1)
plt.plot(magnetic_x, magnetic_y, label='Raw Data', color='gray', marker='o', markersize=2, alpha=0.5)
plt.plot(calibrated_x, calibrated_y, label='Hard-Iron Calibrated', color='blue', marker='x', markersize=2)
plt.title('Raw vs Hard-Iron Calibrated')
plt.xlabel('Magnetic X (Gauss)')
plt.ylabel('Magnetic Y (Gauss)')
plt.legend()
plt.grid()
plt.axis('equal')

# Hard-Iron vs Normalized Soft-Iron Calibrated
plt.subplot(1, 2, 2)
plt.plot(magnetic_x, magnetic_y, label='Raw Data', color='gray', marker='o', markersize=2, alpha=0.5)
plt.plot(calibrated_x, calibrated_y, label='Hard-Iron Calibrated', color='blue', marker='x', markersize=2)
plt.plot(normalized_final_calibrated_x, normalized_final_calibrated_y, label='Normalized Soft-Iron Calibrated', color='green', marker='+', markersize=2)
plt.title('Raw vs Hard and Soft Iron Calibrated(Final Calibration)')
plt.xlabel('Magnetic X (Gauss)')
plt.ylabel('Magnetic Y (Gauss)')
plt.legend()
plt.grid()
plt.axis('equal')
plt.tight_layout()
plt.show()