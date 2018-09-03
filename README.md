# Frankie Bot
Frankie Bot is named after Mary Shelley's Frankenstein. 

Frankie currently uses a Jetson TX2 as the on board computer with the ROS operating system, YDLIDAR X4, 9DoF Razor IMU M0, Zedd Camera, Arduino microcontroller, VESC speed controller, default RC motor, chasis, and wheels. A SSD card was used with the Jetson TX2 for more storage capacity. 

1. Install Jetpack: 
The Jetson TX2 is an ARM architecture and comes flashed with a linux flavored operating system. If you decide to utilize an external SSD then you will need to flash the SSD with Jetpack. 

2. Patch Jetson TX2 and Configure Libraries
The Jetson TX2 is an ARM archietecture computer which means that severl crucial libraries don't compile with the computer. This package, https://github.com/rbonghi/jetson_easy, is an easy one step package to update the distributon, patch the operating system for known errors, install eseential libraries, and will also install ROS. 

3. Install ROS
If you did not install ROS from the jetson_easy package then you will need to follow these instructions, http://wiki.ros.org/action/show/kinetic/Installation/Ubuntu?action=show&redirect=kinetic%2FInstallation%2FUbuntuARM, in order to setup ROS for your on board computer. We utilized the kinetic version of ROS. There should be no incompatabilities with using Indigo. 

4. Configure Base Controller





