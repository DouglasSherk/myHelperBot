/* 
 * rosserial::std_msgs::Float64 Test
 * Receives a Float64 input, subtracts 1.0, and publishes it
 */

#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

const int ledPin = 13;
int ledState = LOW;

void messageCb(const std_msgs::String& msg) {
  ledState = (ledState == LOW ? HIGH : LOW);
  digitalWrite(ledPin, ledState);
}

std_msgs::String test;
ros::Subscriber<std_msgs::String> s("chatter", &messageCb);
ros::Publisher p("my_topic", &test);

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(p);
  nh.subscribe(s);
}

void loop()
{
  //test.data = 'ham';
  //p.publish( &test );
  nh.spinOnce();
  delay(10);
}
