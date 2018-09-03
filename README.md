# Frankie Bot
Frankie Bot is named after Mary Shelley's Frankenstein. 

Frankie Hardware
Jetson TX2 as the on board computer
YDLIDAR X4
9DoF Razor IMU M0 
Zedd Camera
Arduino Mega Microcontroller
VESC Speed Controller
Default RC motor, Chasis, and Wheels 
256 GB SSD Card was used with the Jetson TX2 for more storage capacity for data


1. Jetpack: 
The Jetson TX2 is an ARM architecture and comes flashed with a linux flavored operating system. If you decide to utilize an external SSD then you will need to flash the SSD with Jetpack, https://developer.nvidia.com/embedded/jetpack. 

2. Patch Jetson TX2 and Configure Libraries
The Jetson TX2 is an ARM archietecture computer which means that severl crucial libraries don't compile with the computer. This package, https://github.com/rbonghi/jetson_easy, is an easy one step package to update the distributon, patch the operating system for known errors, install eseential libraries, and will also install ROS. 

3. ROS Installation
If you did not install ROS from the jetson_easy package then you will need to follow these instructions, http://wiki.ros.org/action/show/kinetic/Installation/Ubuntu?action=show&redirect=kinetic%2FInstallation%2FUbuntuARM, in order to setup ROS for your on board computer. We utilized the kinetic version of ROS. There should be no incompatabilities with using Indigo. 

4. Base Controller Setup
The ROS Naviagation stack publishes to the /cmd_vel topic which is subscribed to by the base controller node and converts the command into pulse width modulation signals that can be read by the motor. The base controllwer is located on an arduino mega and can  and receives the messages through the rosserail_python package, http://wiki.ros.org/rosserial_python. The base controller is a ROS node that takes the messages from /cmd_vel as a callback and converts the values which range from -0.5 to 0.5 for linear and angular velocity and maps them to values between max and min throttle and steering. The base controller needs to be configured for min and max throttle and steering based your specific motor and servo. If not configured properly then it can burn out the motor or the servo. These numbers also need to be adjusted based on the payload your robot is carrying. 

5. VESC speed controller 
A Brushless DC (BLDC) motor is controlled by an electronic speed controller (ESC). The Vedder Electronic Speed Controller (VESC) is based on the open source speed controller by Benjamin Vedder. The benefits of the VESC is it allows for forward and reverse control, regulates the current passing through the motor, temperature, as well as other benefits with sensored motors that can be utilized in odometry that were not used in this project. In order to get started with and configure the VESC then you will need to first download the software here, https://vesc-project.com/. You will need to make an account and "order" the software, $0.00, which is sent via email. You can configure the VESC by going to "Motor Setting" and set the motor to BLDC. Set the appropriate current, voltage, rpm, wattage, and temperature for your motor. Then go to App Settings and set "App to Use" to PPM. PPM is similar to pulse width modulation (PWM) and PPM will be able to take the PWM signals from the arduino and output them to motor speed commands. Then naviagate to "PPM" and under general set the control type to "Current No Reverse" for singular direction output and "Current" to allow for the motor to run in both directins. After go to "Mapping" and send the VESC PWM singles from the arduino and configure the Pulselength Start, End, and Center outputs. Apply the new settings to the VESC. 

6. Dead Reckoning
Dead reckoning is the process of estimating the position/state of the robot by using previously determined position and estimating the new position based on different odometry sources. Frankie uses to current sources of odometry, an inertial measurement unit (IMU) and Visual Odometry (VO). Each odometry source accumlulates drift in measurements. We fuse the data from the sensors using an Extended Kalman Filter (EKF) to more accurately predict position. 

7.Sparkfun 9 DoF IMU
This 9DoF (9 Degrees of Freedom) IMU uses three 3-axis sensors—an accelerometer, gyroscope and magnetometer that give it the ability to sense linear acceleration, angular rotation velocity and magnetic field vectors. In order to use the IMU you first need to upload the correct firmware to the device, https://github.com/sparkfun/9DOF_Razor_IMU/tree/v30/Firmware/_9DoF_Razor_M0_Firmware. After uploading the firmware to the IMU you need to configure your onboard computer with the razor ROS driver, http://wiki.ros.org/razor_imu_9dof. After installing the driver calibrate the IMU as described in the wiki. The node publishes publishes a topic /imu, but the topic is remapped to /imu_data in order to sync with the EKF.

8. Zedd Camera
The Zedd camera is a stereo camera that utilizes two cameras depth perception, positional tracking, and spatial mapping. The Zedd camera can make point clouds of the environment which can be used for Real Time Apperance Based Mapping (RTAB-Mapping). Additionally, Frankie will be utilize the Zedd camera for object recognition. In order to configure the Zedd camera you need to first ensure that you have a computer with a GPU and CUDA. Although, a GPU and CUDA is installed on the JETSON TX2 it is unable to update the firmware of the Zedd camera. Install the ZED SDK, https://www.stereolabs.com/developers/release/2.5/ and go to Zed Explorer in order to update the firmware as well as test the Zedd camera for other configurations. After configuring the Zedd camera, install the Zedd wrapper, https://github.com/stereolabs/zed-ros-wrapper. The Zedd publishes zedd/odom topic and needs to be remapped to /vo in order to sync with the EKF. 

9. Robot Pose EKF
A kalman filter is an algorithm that uses a joint probability distribution of sensors that contain noise in order to determine and unknown variable, in our case position. The extended kalman filter is used for nonlinear state estimation. In Frankie, the Robot Pose EFK subscribes to /imu_data from the imu and /vo from the zedd camera to fuse them to produce a single state estimation that is published to /odom_combined. Robot Pose EKF comes installed with the navigation stack and more information on setup and configuration can be found here,http://wiki.ros.org/robot_pose_ekf. In future iterations, the robot_localization node, http://wiki.ros.org/robot_localization, can be utilized which also does non linear state estimation through an arbitrary amount of sensors in comparison to the three, IMU, VO, Wheel Encoders, that the robot_pose_ekf is constrained by. Additionally, a nonscented kalman filter node can be programmed for additional accuracy and comparison. 


10. YDLIDAR X4
The YDLidar 

11. TF

12. Simultaneous Localization and Mapping

13. Gmapping

14. Autonomous Navigation

15. AMCL
In order to help compensate for the drift that occurs in odometry, Adaptive Monte Carlo Localization (ACML) is utilized to better determine location. ACML is a type of localization that uses a laser and particle filter to determine location against a known map.  Probabilistic Robotics by Thrun, also known as the godfather of self driving cars, https://docs.ufpr.br/~danielsantos/ProbabilisticRobotics.pdf, can go into depth on various localization tehcniques. Frankie, currently uses AMCL, but will also implement RTAB mapping with the Zedd Camera. 

16. Teb Local Planner



In Progress:
Mapping with RTAB and Hector SlAM to compare the results against Gmapping.
Add GPS Module to EKF
YOLO2 Node for object recognition that publishes stop/pause commands to /cmd_vel node
Wheel Encoders for better dead reckoning
Custom planner that in an uncertain environment with large tolerances will use, "simple and
best-effort" vs "complex and precise"
3D Lidar
Unscented Kalman Fitler 
Deep Learning Module to that takes images and lidar that checks output steering and throttle commands against the ones sent by the navigation stack
The use of Real Time Operating Systems





