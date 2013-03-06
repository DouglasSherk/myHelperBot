#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

#include "MC33926MotorShield.h"
#include "Encoder.h"

#include "MotorController.h"
#include "NewPing.h"

#define SONAR_NUM     6 // Number or sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define MOTOR_INTERVAL 100 // Milliseconds between speed corrections

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.
unsigned long gMotorTimer;

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(38, 39, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(40, 41, MAX_DISTANCE),
  NewPing(42, 43, MAX_DISTANCE),
  NewPing(44, 45, MAX_DISTANCE),
  NewPing(46, 47, MAX_DISTANCE),
  NewPing(48, 49, MAX_DISTANCE),
};

ros::NodeHandle nh;

MotorController mcL(30, 22, 24, 2, 26, 28, 10, 11, 5);
MotorController mcR(31, 23, 25, 3, 27, 29, 8, 9, 4);

void motorRCallback(const std_msgs::Int32& speed) {
  mcR.setSpeed(-speed.data);
}

void motorLCallback(const std_msgs::Int32& speed) {
  mcL.setSpeed(speed.data);
}

std_msgs::String gUltrasonic;
ros::Subscriber<std_msgs::Int32> ros_sMotorR("motorR", &motorRCallback);
ros::Subscriber<std_msgs::Int32> ros_sMotorL("motorL", &motorLCallback);
ros::Publisher ros_pUltrasonic("ultrasonic", &gUltrasonic);

void setup()
{
  mcR.init();
  mcL.init();
  nh.initNode();
  nh.advertise(ros_pUltrasonic);
  nh.subscribe(ros_sMotorR);
  nh.subscribe(ros_sMotorL);

  unsigned long time = millis();

  pingTimer[0] = time + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
  gMotorTimer = time + 75;
}

void loop() {
  unsigned int time = millis();

  // Ultrasonic timed code.
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (time >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }

  mcR.updateEncoder();
  mcL.updateEncoder();

  // Motor timed code.
  if (time - gMotorTimer > MOTOR_INTERVAL) {
    mcR.periodicUpdate(time - gMotorTime);
    mcL.periodicUpdate(time - gMotorTime);
    gMotorTimer = time;
  }

  // The rest of your code would go here.
  nh.spinOnce();
}

void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  static char buf[512];
  buf[0] = 0;

  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    if (buf[0]) {
      sprintf(&buf[strlen(buf)], ", %d", cm[i]);
    } else {
      sprintf(buf, "%d", cm[i]);
    }
  }

  gUltrasonic.data = buf;

  ros_pUltrasonic.publish(&gUltrasonic);
}
