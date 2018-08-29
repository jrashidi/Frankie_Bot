/*
 Base controller for autonomous RC
 Arduino drives a servo and esc
 By Justin Rashidi
 */

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

const float minSteering = 1150;
const float maxSteering = 1600;
const float minThrottle = 700;
const float maxThrottle = 1100;
const float steeringIncrement = 9.0;
float steeringAngle;
float escCommand;
float escThrottle;
float smoothSteering = 1375;
float diffGreat;
float deffLess;

Servo steeringServo;
Servo esc;

ros::NodeHandle nh;

std_msgs::Int32 str_msg;
ros::Publisher base_controller_chatter("base_controller_chatter", &str_msg);

//function to convert twist.msg to a usable number
float fmap(float toMap, float in_min, float in_max, float out_min, float out_max){
  return(toMap - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void driveCB(const geometry_msgs::Twist& twistMsg){
  if(twistMsg.angular.z > 0.0 ){
   steeringAngle = fmap(twistMsg.angular.z, 0.0, 0.5, smoothSteering, maxSteering);
  }

  else if(twistMsg.angular.z < 0.0 ){
   steeringAngle = fmap(twistMsg.angular.z, 0.0, -0.5, smoothSteering, minSteering);
  } 
 
  else if (twistMsg.angular.z == 0.0) {
   steeringAngle = smoothSteering;
  }

 //check steering angle to make sure it is not above max
 if(steeringAngle > maxSteering){
   steeringAngle = maxSteering;
 }

  steeringServo.writeMicroseconds(steeringAngle);

   if(twistMsg.linear.x >= 0.25){
   escCommand = (float)fmap(twistMsg.linear.x, 0.25, 0.5, 750.0, maxThrottle);
 } else {
   escCommand = (float)fmap(twistMsg.linear.x, 0.0, 0.25, 0, 750.0);
 }

 if (escCommand < minThrottle){
   escCommand = minThrottle;
 }

 if(escCommand > maxThrottle){
   escCommand = maxThrottle;
 }

 esc.writeMicroseconds(escCommand);
}

ros::Subscriber<geometry_msgs::Twist>driveSubscriber("cmd_vel", &driveCB);

void setup(){
  Serial.begin(9600);
  nh.initNode();
  nh.advertise(base_controller_chatter);
  nh.subscribe(driveSubscriber);
  esc.attach(9);
  steeringServo.attach(10);
  esc.writeMicroseconds(650);
  steeringServo.writeMicroseconds(smoothSteering);
}

void loop(){
  //refreshes node
  nh.spinOnce();
  delay(1);
}