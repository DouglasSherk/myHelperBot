/* 
 * rosserial::std_msgs::Float64 Test
 * Receives a Float64 input, subtracts 1.0, and publishes it
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

#include "MC33926MotorShield.h"

ros::NodeHandle nh;

MC33926MotorShield m1(21, 23, 25, 3, 27, 29);
MC33926MotorShield m2(20, 22, 24, 2, 26, 28);

const int ledPin = 13;
int ledState = LOW;

void messageCb(const std_msgs::String& msg) {
  ledState = (ledState == LOW ? HIGH : LOW);
  digitalWrite(ledPin, ledState);
}

void motorACallback(const std_msgs::Int32& speed) {
  ledState = (ledState == LOW ? HIGH : LOW);
  digitalWrite(ledPin, ledState);
  m1.setSpeed(speed.data);
}

void motorBCallback(const std_msgs::Int32& speed) {
  m2.setSpeed(speed.data);
}

std_msgs::String test;
ros::Subscriber<std_msgs::String> s("chatter", &messageCb);
ros::Subscriber<std_msgs::Int32> ros_sMotorA("motorA", &motorACallback);
ros::Subscriber<std_msgs::Int32> ros_sMotorB("motorB", &motorBCallback);
ros::Publisher p("my_topic", &test);

void setup()
{
  m1.init();
  m2.init();
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(p);
  nh.subscribe(s);
  nh.subscribe(ros_sMotorA);
  nh.subscribe(ros_sMotorB);
}

void loop()
{
  //test.data = 'ham';
  //p.publish( &test );
  nh.spinOnce();
  delay(10);
}
