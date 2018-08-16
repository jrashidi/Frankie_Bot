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
#include <geometry_msgs/TwistStamped.h>

const float minSteering = 1200;
const float maxSteering = 1800;
const float minThrottle = 0;
const float maxThrottle = 1680;
const float steeringIncrement = 9.0;
float escCommand;
float escThrottle;
float smoothSteering = 1500;
float diffGreat;
float deffLess;

Servo steeringServo;
Servo esc;

ros::NodeHandler nh;

std_msgs::Int32 str_msg;
ros::Publisher chatter("base_controller_chatter", str_msgs);

//function to convert twist.msg to a usable number
float fmap(float toMap, float in_min, float in_max, float out_min, float out_max){
  return(toMap - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//receives a twist.angular.z message and translates it into a useable servo angle
void writeSteeringservo(msg){

  float steeringAngle = fmap(msg, 0.0, 1.0, minSteering, maxSteering);
 //check steering angle to not be below min
 if(steeringAngle < minSteering){
   steeringAngle = minSteering;
 }
 //check steering angle to make sure it is not above max
 if(steeringAngle > maxSteering){
   steeringAngle = maxSteering;
 }

  steeringServo.writeMicrosecond(steeringAngle);

 }

//receives a twist.linear.x message and translates it into a useable motor speed
void writeEsc(msg){
 if(msg >= 0.5){
   escCommand = (float)fmap(msg, 0.5, 1, 1500.0, maxThrottle);
 } else {
   esc = (float)fmap(msg, 0.0, 0.5, 0, 1500.0);
 }

 if (escCommand < minThrottle){
   escCommand = minThrottle;
 }

 if(escCommand > maxThrottle){
   escCommand = maxThrottle;
 }

 esc.writeMicrosecond(escCommand);
}

void driveCB(const geometry_msgs::TwistStamped& twistMsg){
   writeSteeringservo(twistMsg.twist.angular.z);
   writeEsc(twistMsg.twist.linear.x);
}

ros::Subscriber<geometry_msgs::TwistStamped>driveSubscriber("cmd_vel", &driveCB);

void Setup(){
  Serial.begin(9600);
  nh.initNode();
  nh.advertise(base_controller_chatter);
  nh.subscribe(driveSubscriber);
  esc.attach(9);
  steeringServo.attach(10);
  steeringServo.writeMicrosecond(1500);
}

void Loop(){
  //refreshes node
  nh.spinOnce();
  delay(1);
}