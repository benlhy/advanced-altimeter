#include <Arduino.h>
#include <SD.h>
#include <Wire.h>
#include "SparkFunMPL3115A2.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Defines
#define OLED_RESET 4
#define XPOS 0
#define YPOS 1
#define DELTAY 2
#define UPDATERATE 10
#define BNO055_SAMPLERATE_DELAY_MS 1
#define LOOP2_TIME 8
#define LOOP3_TIME 200
#define IMPACT_G 20
#define MIN_H 50 //minimum height above ground, 50ft

#define PIN_X A0
#define PIN_Y A1
#define PIN_Z A2
#define GREEN_LED 15
#define RED_LED 16
#define CS_PIN 11
#define ENERGETICS_PIN_1 A3
#define ENERGETICS_PIN_2 A5
#define TOGGLE_PIN 27


#define VOLTAGE_M 1587 // measured
#define ACC_CALIBRATION 3600/4096*200/VOLTAGE_M // 12 bits, 3.6V reference


MPL3115A2 baro;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
File currentFile;


// Globals
int heartbeat = 0;
int prev_heartbeat = 0;
volatile int G_altm;
int G_ground_altm, G_apogee;
int G_tempC, G_batt;
int G_x, G_y, G_z;
volatile int G_ex, G_ey, G_ez;
float G_High_x, G_High_y, G_High_z;
int red_state = 0;
int green_state = 0;
int in_error = 0;
int record_lock = 0; // flag to lock recording
int activate_1 = 0, activate_2 = 0;

int launched = 0;
int landed = 0;
int highest_point = 0;

boolean recording = false;

int baro_on, bno_on; // variables to track status
unsigned long currMillis, prevMillis, startMillis;
unsigned long flushTime; // file flush
unsigned long loop2Millis = millis(), loop3Millis = millis();
volatile long testMillis, diffMillis;


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
