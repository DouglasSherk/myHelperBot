#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

#include "MC33926MotorShield.h"
MC33926MotorShield motorL(30, 22, 24, 2, 26, 28); //left
MC33926MotorShield motorR(31, 23, 25, 3, 27, 29); //right

#include "Timer.h"
Timer t;

#include "Encoder.h"
Encoder enL(10,11,5);
Encoder enR(8,9,4);

#include "MotorController.h"
MotorController mcL(motorL, enL);
MotorController mcR(motorR, enR);

#include "DualMotorController.h"
DualMotorController dmc(mcL, mcR);

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

int leftSpeed, rightSpeed;

void motorRCallback(const std_msgs::Int32& speed) {
  leftSpeed = speed.data;
  dmc.setSpeed(leftSpeed, rightSpeed);
}

void motorLCallback(const std_msgs::Int32& speed) {
  rightSpeed = speed.data
  dmc.setSpeed(leftSpeed, rightSpeed);
}

std_msgs::String gUltrasonic;
ros::Subscriber<std_msgs::Int32> ros_sMotorR("motorR", &motorRCallback);
ros::Subscriber<std_msgs::Int32> ros_sMotorL("motorL", &motorLCallback);
ros::Publisher ros_pUltrasonic("ultrasonic", &gUltrasonic);

void setup()
{
  motorL.init();
  motorR.init();
  enL.init();
  enR.init();
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

  dmc.updateEncoders();

  // Motor timed code.
  if (time - gMotorTimer > MOTOR_INTERVAL) {
    mcR.periodicUpdate(time - gMotorTimer);
    mcL.periodicUpdate(time - gMotorTimer);
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
