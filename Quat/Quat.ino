#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Quaternion.h"

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();

Quaternion q1 = Quaternion();
Quaternion q2 = Quaternion();

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  delay(1000);
  imu::Quaternion quat = bno.getQuat();
  q1.a = quat.w();
  q1.b = quat.x();
  q1.c = quat.y();
  q1.d = quat.z();

  
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data */
  /*
  Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print(" Z: ");
  Serial.print(euler.z());
  Serial.print("\t\t");
  */

  
  // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  /*
  Serial.print("qW: ");
  Serial.print(quat.w(), 4);
  Serial.print(" qX: ");
  Serial.print(quat.y(), 4);
  Serial.print(" qY: ");
  Serial.print(quat.x(), 4);
  Serial.print(" qZ: ");
  Serial.print(quat.z(), 4);
  Serial.print("\t\t");
  */

  q2.a = quat.w();
  q2.b = quat.x();
  q2.c = quat.y();
  q2.d = quat.z();
  Quaternion q3 = q1.conj()*q2; // change in q1 to q2.

  Serial.print("qW: ");
  Serial.print(q3.a, 4);
  Serial.print(" qX: ");
  Serial.print(q3.b, 4);
  Serial.print(" qY: ");
  Serial.print(q3.c, 4);
  Serial.print(" qZ: ");
  Serial.print(q3.d, 4);
  Serial.print("\t\t");


  
  Quaternion v = Quaternion(); // create an empty vector.
  v.a = 0;
  v.b = 0;
  v.c = 1;
  v.d = 0;
  Quaternion u = q3.rotate(v);
  Serial.print("qW: ");
  Serial.print(u.a, 4);
  Serial.print(" qX: ");
  Serial.print(u.b, 4);
  Serial.print(" qY: ");
  Serial.print(u.c, 4);
  Serial.print(" qZ: ");
  Serial.print(u.d, 4);
  Serial.print("\t\t");
  Serial.print("Angle: ");
  Serial.println(acos(u.b*v.b+u.c*v.c+u.d*v.d));
 
  
  
  

  /* Display calibration status for each sensor. */
   /*
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);
  */

  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void quat_math(){
  
}



