# Frankie Bot
Frankie Bot is named after Mary Shelley's Frankenstein. 

Frankie currently uses a Jetson TX2 as the on board computer with the ROS operating system, YDLIDAR X4, 9DoF Razor IMU M0, Zedd Camera, Arduino microcontroller, VESC speed controller, default RC motor, chasis, and wheels. A SSD card was used with the Jetson TX2 for more storage capacity. 

1. Jetpack: 
The Jetson TX2 is an ARM architecture and comes flashed with a linux flavored operating system. If you decide to utilize an external SSD then you will need to flash the SSD with Jetpack. 

2. Patch Jetson TX2 and Configure Libraries
The Jetson TX2 is an ARM archietecture computer which means that severl crucial libraries don't compile with the computer. This package, https://github.com/rbonghi/jetson_easy, is an easy one step package to update the distributon, patch the operating system for known errors, install eseential libraries, and will also install ROS. 

3. ROS Installation
If you did not install ROS from the jetson_easy package then you will need to follow these instructions, http://wiki.ros.org/action/show/kinetic/Installation/Ubuntu?action=show&redirect=kinetic%2FInstallation%2FUbuntuARM, in order to setup ROS for your on board computer. We utilized the kinetic version of ROS. There should be no incompatabilities with using Indigo. 

4. Base Controller Setup
The ROS Naviagation stack publishes to the /cmd_vel topic which is subscribed to by the base controller node and converts the command into pulse width modulation signals that can be read by the motor. The base controllwer is located on an arduino mega and can  and receives the messages through the rosserail_python package, http://wiki.ros.org/rosserial_python. The base controller needs to be configured for min and max throttle and steering based your specific motor and servo. If not configured properly then it can burn out the motor or the servo. These numbers also need to be adjusted based on the payload your robot is carrying. 

5. VESC speed controller 
A Brushless DC (BLDC) motor is controlled by an electronic speed controller (ESC). The Vedder Electronic Speed Controller (VESC) is based on the open source speed controller by Benjamin Vedder. The benefits of the VESC is it allows for forward and backward control, control of the current passing through the motor, as well as other benefits with sensored motors that can be utilized in odometry that were not used in this project. 

In order to get started with and configure the VESC then you will need to first download the software here, https://vesc-project.com/. You will need to make an account and "order" the software, $0.00, which is sent via email. You can configure the VESC by 

6. Sparkfun 9 DoF IMU

7. Zedd Camera

8. Robot Pose EKF 

9. YDLIDAR X4

10. TF 

11. Creating a Map

12. Autonomous Navigation

Coming Soon:
YOLO2 Node for object recognition that publishes stop commands to /cmd_vel node





