GPS Data Acquisition, Driver Development, and Analysis Project

This project focuses on creating a complete GPS data acquisition and analysis pipeline using a USB-based GNSS puck sensor. The goal was to design a system capable of collecting, processing, and analyzing real-time GPS data for applications in robotics and autonomous systems. Throughout the project, I encountered and addressed challenges related to signal accuracy, environmental interference, and noise in GPS readings.

This project involved multiple key phases:

1. Device Setup:
   The first step was to configure the USB GNSS puck sensor for serial communication. Using system commands, I identified the correct serial device and adjusted its read/write permissions. After configuring the sensor settings with minicom, I verified that data from the GPS module could be saved for further use.
2. Custom Driver Development:
   I developed a Python-based ROS2 driver (driver.py) to handle data acquisition from the GNSS puck. This driver:
   1. Reads NMEA $GPGGA messages from the puck, extracting key fields like latitude, longitude, altitude, and timestamp.
   2. Converts GPS coordinates to UTM (Universal Transverse Mercator) using the utm Python library.
   3. Publishes the data as a custom ROS2 message, with fields including:
      1. Latitude, longitude, altitude
      2. UTM easting, UTM northing
      3. Zone and letter information
      4. ROS Header with GPS timestamp for synchronization in multi-sensor systems
   4. To ensure modularity, I developed a ROS2 launch file (gps_launch.py). This file automates the process of running the driver with a specified port, simplifying sensor integration on robots with multiple sensors.
3. Data Collection and Analysis
   I conducted two key data collection experiments:
   1. Stationary Data : I collected data at a single outdoor location for 10 minutes. The GPS coordinates fluctuated significantly despite the device being stationary. Environmental factors such as signal reflections from buildings and trees caused these errors.
      1. Key Findings: Plots showed clear evidence of coordinate drift. On a map, the drift was less noticeable due to scale but could still impact high-precision applications.
   2. Moving Data: I recorded data while walking in a straight line for several meters. The path data remained closer to the true trajectory compared to stationary data. When in motion, signals from multiple satellites improved overall accuracy and reduced error.
      1. Observations: GPS errors were more constrained during motion, likely due to continuous updates from satellite corrections. Error behavior differed significantly between static and dynamic scenarios.


Technologies Used:
1. Python: Custom ROS2 driver and data processing
2. ROS2: Integration of GPS sensor with modular nodes and launch files
3. GNSS Hardware: USB GNSS puck sensor for data collection
4. Matlab/Python: Statistical analysis and visualization of GPS data
5. Linux: Terminal-based device setup and command-line tools for data capture

This project provided hands-on experience with:
1. GPS hardware and data handling techniques.
2. Designing a modular ROS2 package for sensor integration.
3. Understanding GPS errors and the impact of environmental factors.
4. Collecting and analyzing data to improve navigation accuracy in robotics applications.
