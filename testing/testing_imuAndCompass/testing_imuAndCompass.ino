//imu
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define LED_PIN 13
bool blinkState = false;

//compass
#include <LSM303.h>

LSM303 compass;


void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
    
    //compass
    compass.init();
    compass.enableDefault();
    
    // Calibration values. Use the Calibrate example program to get the values for
    // your compass.
    //compass.m_min.x = -548; compass.m_min.y = -604; compass.m_min.z = -491;
    //compass.m_max.x = +344; compass.m_max.y = +294; compass.m_max.z = +483;
    compass.m_min.x = 70; compass.m_min.y = -218; compass.m_min.z = -191;
    compass.m_max.x = 183; compass.m_max.y = 31; compass.m_max.z = 66;
}

void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    // display tab-separated accel/gyro x/y/z values
    Serial.print("a/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.print(gz);

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

    compass.read();
    int heading = compass.heading((LSM303::vector){0,-1,0});
    Serial.print("\t");
    Serial.print("heading:");
    Serial.println(heading);
    delay(100);
}
