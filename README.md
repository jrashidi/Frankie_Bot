# Frankie Bot
Frankie Bot is named after Mary Shelley's Frankenstein. 

This wiki is mix between education and tutorial in hopes that others will be able to quickly learn and assemble the disjointed ROS wiki. Prior to starting to assemble Frankie you should do all of the beginner tutorials on this page, http://wiki.ros.org/ROS/Tutorials.

Frankie Hardware

Jetson TX2 as the on board computer

YDLIDAR X4

9DoF Razor IMU M0 

Zedd Camera

Arduino Mega Microcontroller

VESC Speed Controller

Default RC motor, Chasis, and Wheels 

256 GB SSD Card was used with the Jetson TX2 for more storage capacity for data


1. Jetpack

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

The YDLidar is a 360 degree 2D lidar that is used for mapping and object detection. The YDLidar uses triangulation instead of time of flight as the means of calculating distance, https://www.acuitylaser.com/measurement-principles. In order to setup the YDLidar you need to make the package in your src folder, https://github.com/EAIBOT/ydlidar. After making the package then you need to check the usb port that the lidar is connected to in order to set the port in the launch file. 

11. TF

The Transform ROS package helps to keep track of all coordinate frames of the robot, http://wiki.ros.org/tf. All of Frankie transforms were put in using the static_transform_publisher in order to isolate the publishers, http://wiki.ros.org/tf#static_transform_publisher. 

In order to adjust a static_transform_publisher you need to adjust the args of the node. The args are the x,y,z and the roll, pitch, and yaw of the rotation. This publisher tells ROS that the base_link is 0.098 meters above the base_footprint. Without this transform, the robot would hit any obstacle that is below 0.098 meters because it would have no reference point to the ground. 
<!-- Publish static transform from base_footprint to base_link -->
  <node pkg="tf" type="static_transform_publisher" name="base_footprint_to_base_link" args="0 0 0.098 0 0 0  /base_footprint /base_link  100"/>
  
In order to setup Frankie then you need to adjust a few tf publishers.

Frankie has the following transformations:
map -> odom 
This transformation is published by the AMCL node and does not need to be adjusted. This node is often confusing because it would seem that this transformation should publish to the base_link frame, but since children frames can only have one parent this points to the odom frame. This map frame is a fixed frame of reference, but is non-continous and can have discrete jumps in the position of the robot. While the odom frame is a continous frame of reference from dead reckoning that can drift over time. 

odom -> base_footprint
This transform is the relationship between the robot's current pose from its origin and the base_footprint. The base_footprint differs from the base_link as it is the a shadow of the base_link reflected on the floor. This base_footprint helps in obstacle avoidance. This transform is published by robot_pose_efk package. This frame of reference also does not need to be adjusted. 

base_footprint -> base_link
This transform is the static relationship between the center of the robot base and the floor. This needs to be adjusted in the minimal.launch file. You need to measure the distance between the floor and the center of the robot base. The measured distance on the third value found in "args" parameter is the z direction which is the upward directon. 

base_link -> laser
This transform is the static relationship between the lidar and the robot's base. This publisher can be found in the include/laser.launch file.

base_link -> camera
This transform is the static relationship between the camera and the robot's base. This publisher can be found in the include/zedd.launch file.

base_link -> imu
This transform is the static relationship between the imu and the robot's base. This publisher can be found in the include/imu.launch file.

12. Costmap Common Params Footprint

In the costmap_common_params.yaml you need to set the robot footprint for the robot.  The footprint of the robot is the area that the robot occupies and is mapped in x,y coordinates from the center of the robot: [[-x, -y], [-x, y], [x, y], [x, -y]]. For simplicity Frankie used a circular radius instead of a box footprint this will be updated at a later date. 

12. Joy/Teleop_Twist_Joy

In order to control the robot while not in autonomous mode then you need to connect Frankie to a remote controller. We used the wireless logitech game controller. In order to use this controller then you need to install two ROS packages, the joy node, http://wiki.ros.org/joy. The joy package registers the joystick comamnds from the controller. The Teleop package, http://wiki.ros.org/teleop_twist_joy,  subscribes to the joy topic and will convert the joy outputs to twist_msgs that are published to the cmd_vel topic and is read by the base controller. 

13. Simultaneous Localization and Mapping

Simultaneous localiation and mapping (SLAM) is the technique of mapping a previously unknown location. There are several techniques for SLAM: RTAB-Mapping, Hector SLAM, ORB SLAM, KARTO SLAM, HDL SLAM, and Gmapping. For Frankie we used gmapping for the SLAM technique. Gmapping uses odometery and laser scans and performs better in ambiguous areas where it can rely on odom data.

You need two terminals to launch SLAM. 

Run "roslaunch frankie_bot minimal.launch" then "roslaunch frankie_bot slam.launch"

14. AMCL

In order to help compensate for the drift that occurs in odometry, Adaptive Monte Carlo Localization (ACML) is utilized to better determine location. ACML is a type of localization that uses a laser and particle filter to determine location against a known map.  Probabilistic Robotics by Thrun, also known as the godfather of self driving cars, https://docs.ufpr.br/~danielsantos/ProbabilisticRobotics.pdf, can go into depth on various localization tehcniques. Frankie, currently uses AMCL, but will also implement RTAB mapping with the Zedd Camera. 

15. Timed Elastic Band Local Planner

Timed Elastic Band (TEB) Planner, http://wiki.ros.org/teb_local_planner,  is the local planner that was used with the navigation stack. The TEB Planner was choosen because of the use of the planner with Ackermann steering. TEB Local Planner uses a Timed Elastic Band approach to the local trajectory of the robot. Where the path can be stretched in one direction or another in order to allow for dynamic planning as well as object avoidance. The other planner that was considered is the Dynamic Window Approach (DWA) Planner,http://wiki.ros.org/dwa_local_planner, but additional modifications would need to be made in order to adapt the package to ackermann steering.  

16. Autonomous Navigation

Finally, Frankie should be able to navigate autonomously. In order to better understand all the nodes and configurations of the navigation stack read the ROS documentation, http://wiki.ros.org/navigation.

You need two terminals to launch SLAM. 

Run "roslaunch frankie_bot minimal.launch" then "roslaunch frankie_bot navigate.launch"

To send goals to Frankie open up RVIZ and send goals to him in the known map. 

------------------------------------------------------------------------------------------------------------------------------

In Progress:

Mapping with RTAB and Hector SLAM to compare the results against Gmapping.

Add GPS Module to EKF

YOLO2 Node for object recognition that publishes stop/pause commands to /cmd_vel node

Wheel Encoders for better dead reckoning

Custom planner that in an uncertain environment with large tolerances will use, "simple and best-effort" vs "complex and precise"

3D Lidar

Unscented Kalman Fitler

Deep Learning Module to that takes images and lidar that checks output steering and throttle commands against the ones sent by the navigation stack

The use of Real Time Operating Systems





