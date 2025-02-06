# **GPS Data Acquisition, Driver Development, and Analysis Project**

This project focuses on creating a complete GPS data acquisition and analysis pipeline using a USB-based GNSS puck sensor. The goal was to design a system capable of collecting, processing, and analyzing real-time GPS data for applications in robotics and autonomous systems. Throughout the project, I encountered and addressed challenges related to signal accuracy, environmental interference, and noise in GPS readings.

## **Project Overview**

This project involved multiple key phases:
1. **Device Setup**
2. **Custom Driver Development**
3. **Data Collection and Analysis**
4. **Integration with ROS2**

By completing these phases, I was able to study both static and dynamic behavior of GPS signals, observe sources of error, and refine data handling techniques.

---

### **Phase 1: Device Setup and Configuration**

The first step was to configure the USB GNSS puck sensor for serial communication. Using system commands, I identified the correct serial device and adjusted its read/write permissions. After configuring the sensor settings with `minicom`, I verified that data from the GPS module could be saved for further use.

**Commands used:**
- `$ ls –lt /dev/tty* | head`: Identify device file
- `$ chmod 666 /dev/ttyUSB2`: Set permissions for device access
- `minicom`: Read GPS output and save to `gps-data.txt`

---

### **Phase 2: Custom Device Driver Development**

I developed a Python-based ROS2 driver (`driver.py`) to handle data acquisition from the GNSS puck. This driver:
- Reads NMEA `$GPGGA` messages from the puck, extracting key fields like latitude, longitude, altitude, and timestamp.
- Converts GPS coordinates to UTM (Universal Transverse Mercator) using the `utm` Python library.
- Publishes the data as a custom ROS2 message, with fields including:
  - Latitude, longitude, altitude
  - UTM easting, UTM northing
  - Zone and letter information
  - ROS `Header` with GPS timestamp for synchronization in multi-sensor systems

To ensure modularity, I developed a ROS2 launch file (`gps_launch.py`). This file automates the process of running the driver with a specified port, simplifying sensor integration on robots with multiple sensors.

---

### **Phase 3: Data Collection**

I conducted two key data collection experiments:

#### **1. Stationary Data**
- I collected data at a single outdoor location for 10 minutes.
- The GPS coordinates fluctuated significantly despite the device being stationary.
- Environmental factors such as signal reflections from buildings and trees caused these errors.
- Correction systems like WAAS (Wide Area Augmentation System) and satellite position delays also contributed to inaccuracies.

##### Key Findings:
- Plots showed clear evidence of coordinate drift.
- On a map, the drift was less noticeable due to scale but could still impact high-precision applications.

---

#### **2. Moving Data**
- I recorded data while walking in a straight line for several meters.
- The path data remained closer to the true trajectory compared to stationary data.
- When in motion, signals from multiple satellites improved overall accuracy and reduced error.

##### Observations:
- GPS errors were more constrained during motion, likely due to continuous updates from satellite corrections.
- Error behavior differed significantly between static and dynamic scenarios.

---

### **Phase 4: Data Analysis and Visualization**

Using Python and Matlab, I analyzed both stationary and walking data. I generated various plots to visualize the data, focusing on:
- Error distribution in GPS readings
- The impact of environmental factors on signal accuracy
- Differences in noise behavior between stationary and moving scenarios

Plots and charts provided insights into the strengths and limitations of GPS navigation, particularly under different conditions.

---

### **Technologies Used**
- **Python**: Custom ROS2 driver and data processing
- **ROS2**: Integration of GPS sensor with modular nodes and launch files
- **GNSS Hardware**: USB GNSS puck sensor for data collection
- **Matlab/Python**: Statistical analysis and visualization of GPS data
- **Linux**: Terminal-based device setup and command-line tools for data capture

---

### **Key Learnings**
This project provided hands-on experience with:
- GPS hardware and data handling techniques.
- Designing a modular ROS2 package for sensor integration.
- Understanding GPS errors and the impact of environmental factors.
- Collecting and analyzing data to improve navigation accuracy in robotics applications.

---

### **Next Steps**
Future improvements could include:
- Implementing real-time error correction algorithms.
- Integrating additional sensors (e.g., IMU) to enhance positioning accuracy.
- Applying advanced filtering techniques, such as Kalman filters, for sensor fusion.

---

Feel free to explore the repository and contribute! I'm always open to feedback, suggestions, and collaborations on similar projects.
