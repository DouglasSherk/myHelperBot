/* 
 * rosserial::std_msgs::Float64 Test
 * Receives a Float64 input, subtracts 1.0, and publishes it
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

#include "MC33926MotorShield.h"
#include "NewPing.h"

#define SONAR_NUM     6 // Number or sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(38, 39, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(40, 41, MAX_DISTANCE),
  NewPing(42, 43, MAX_DISTANCE),
  NewPing(44, 45, MAX_DISTANCE),
  NewPing(46, 47, MAX_DISTANCE),
  NewPing(48, 49, MAX_DISTANCE),
};

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
  m1.setSpeed(speed.data);
}

void motorBCallback(const std_msgs::Int32& speed) {
  m2.setSpeed(speed.data);
}

std_msgs::String test;
std_msgs::String gUltrasonic;
ros::Subscriber<std_msgs::String> s("chatter", &messageCb);
ros::Subscriber<std_msgs::Int32> ros_sMotorA("motorA", &motorACallback);
ros::Subscriber<std_msgs::Int32> ros_sMotorB("motorB", &motorBCallback);
ros::Publisher p("my_topic", &test);
ros::Publisher ros_pUltrasonic("ultrasonic", &gUltrasonic);

void setup()
{
  m1.init();
  m2.init();
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(p);
  nh.advertise(ros_pUltrasonic);
  nh.subscribe(s);
  nh.subscribe(ros_sMotorA);
  nh.subscribe(ros_sMotorB);
  
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
}

void loop() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
  // The rest of your code would go here.
}

void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  //sprintf(gUltrasonic.data, "");
  
  //static char buf[512];
  //buf[0] = 0;

#if 0  
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    if (buf[0]) {
      sprintf(buf, "%s, %d", buf, cm[i]);
    } else {
      sprintf(buf, "%d", cm[i]);
    }
  }
#endif
  
  //sprintf(gUltrasonic.data, buf);
  
  //sprintf(gUltrasonic.data, "%d, %d, %d, %d, %d, %d",
  //        cm[0], cm[1], cm[2], cm[3], cm[4], cm[5]);
  gUltrasonic.data = "15 15 15 15 15 15";
  
  ros_pUltrasonic.publish(&gUltrasonic);
  
  ledState = (ledState == LOW ? HIGH : LOW);
  digitalWrite(ledPin, ledState);
}
