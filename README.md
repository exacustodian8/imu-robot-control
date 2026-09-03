# imu-robot-control
Wearable IIMU-based interface for real-time control of a Universal Robots manipulator.

A wearable human-robot interaction system for real-time control of a Universal Robots manipulator using two inertial measurement units (IMUs).

The system uses two BNO055 sensors to independently control the orientation and translation of the robot end-effector. Sensor data is transmitted through a serial connection, processed in Python, and converted into robot motion commands using the RTDE interface.

The project was developed as part of my Bachelor's Thesis in Systems Engineering at the Faculty of Automatic Control and Computer Engineering, "Gheorghe Asachi" Technical University of Iași.

## Features

- Real-time acquisition and processing of data from two IMU sensors
- Independent control of robot orientation and translation
- Relative orientation calibration
- Euler angle processing and rotation transformations using SciPy
- Deadzone compensation for small unintended movements
- Low-pass filtering for smoother motion
- Velocity and workspace limiting
- Real-time robot control using RTDE
- PyQt5 graphical interface for monitoring sensor data and robot commands
- Live visualization of pitch, roll, and yaw data
