
#include <Arduino.h>
#include <SD.h>
#include <bluefruit.h>
#include <Wire.h>
#include "SparkFunMPL3115A2.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Quaternion.h"

/*
   ToDo:
   1. Add sensing section
   2. Add logic

   Notes:
   1. At fastest rate of data collection, 2min = 400kb ~ 3h launch window = 180 min = 36MB.
   2. Current use at room temp is 25mA.
   3. Stress testing with 105mAh Lipo battery yields 1h 40min (116 min), expected was 4h (240 min) ~ 40%.
   4. Recording can be locked by sending 'l' over bluetooth to the device. Important because the button is easy to hit.
   5. Stress testing with 500mAh Lipo battery yields more than 3 h of battery life.
   6. Pressing the button quickly can cause the system to seize up. Recommend changing pins from 30 to 27
   7. Sometimes the system seizes up, try taking out the SD card and putting it back in.
   8. Maybe try having a long press to stop recording?
   9. Pressing calibration while recording messes things up - fixed by changing variables to volatile
   10. Added degree change parameter - use t and then 2 numbers. Tested on a rig, results are similar to a phone.
   11. There are some issues when rotating the thing, likely due to mounting issues (not mounted straight).
   12. Recommend a fudge factor of 5 degrees more than the desired max tilt.
   13. Recommend adding a z[number], which specifies orientation of mounted altimeter
*/

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

#define VBAT_PIN          (A7)
#define VBAT_MV_PER_LSB   (0.87890625F)   // 3.6V ADC range and 12-bit ADC resolution = 3600mV/4096
#define VBAT_DIVIDER      (0.71275837F)   // 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
#define VBAT_DIVIDER_COMP (1.403F)        // Compensation factor for the VBAT divider


#define PIN_X A0
#define PIN_Y A1
#define PIN_Z A2
#define GREEN_LED 15
#define RED_LED 16
#define CS_PIN 11
#define ENERGETICS_PIN_1 A3
#define ENERGETICS_PIN_2 A5
#define TOGGLE_PIN 27
#define TOGGLE_PIN_2 29

#define VOLTAGE_M 1587 // measured
#define ACC_CALIBRATION 3600/4096*200/VOLTAGE_M // 12 bits, 3.6V reference

// BLE Service
BLEDis  bledis;
BLEUart bleuart;
BLEBas  blebas;

// Software Timer for blinking RED LED
SoftwareTimer blinkTimer;

MPL3115A2 baro;
Adafruit_SSD1306 display(OLED_RESET);
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

volatile int toggle_1 = 0, toggle_2 = 0;
int max_tilt = 15;

int anticipated_apogee;

int launched = 0;
int landed = 0;
int highest_point = 0;

boolean recording = false;

int baro_on, bno_on; // variables to track status
unsigned long currMillis, prevMillis, startMillis;
unsigned long flushTime; // file flush
unsigned long loop2Millis = millis(), loop3Millis = millis();
volatile long testMillis, diffMillis;

float zero_G_y;
float zero_G_z;
float G_acc_x,G_acc_y,G_acc_z;
Quaternion q1 = Quaternion();
Quaternion q2 = Quaternion();
Quaternion v = Quaternion(); // create an empty vector.


volatile int update_zero = 0;

float G_angle;

// Functions
void statusUpdate(void);

void setup()
{

  Serial.begin(115200);
  Serial.println("Bluefruit52 BLEUART Example");
  Serial.println("---------------------------\n");
  bleInit();
  Serial.println("BLE init complete.");

  attachInterrupt(TOGGLE_PIN, toggleHandler, FALLING);
  attachInterrupt(TOGGLE_PIN_2, toggleHandler2, FALLING);




  // Sensor init //
  /*
    if (! baro.begin()) {
    Serial.println("Couldnt find sensor");
    baro_on = 0;
    }
    else {
    baro_on = 1;
    }
  */
  baroInit();
  Serial.println("Baro init complete");

  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("ERROR: BNO055 init!");
    bno_on = 0;
    in_error = 1;
  }
  else {
    Serial.println("BNO055 init complete.");
    bno_on = 1;
    bno.setExtCrystalUse(true);
  }
  if (!SD.begin(11)) {
    Serial.println("ERROR: SD card init!");
    in_error = 1;
  }
  else {
    Serial.println("SD card init complete.");
  }

  analogReadResolution(12); // setup resolution for sensor
  acc_init(); // init ADXL377
  Serial.println("High G init complete.");
  pins_init(); // setup LEDs as outputs
  Serial.println("Led and Energetics init complete.");
  //screenInit();
  // Initialize software timers
  blinkTimer.begin(1000, blink_timer_callback);
  blinkTimer.start();
  Serial.println("Init complete.");
  init_complete();
  /*
    sensors_event_t event;
    bno.getEvent(&event);
    zero_G_y = (int)event.orientation.y; // we only care about the y axis.
    zero_G_z = (int)event.orientation.z;
  */
  imu::Quaternion quat = bno.getQuat();
  q1.a = quat.w();
  q1.b = quat.x();
  q1.c = quat.y();
  q1.d = quat.z();
  v.a = 0; //
  v.b = 1; // x
  v.c = 0; // y
  v.d = 0; // z
  Serial.println("Starting .");


  flushTime = millis();
  startMillis = millis(); // start counting after init complete


}


void loop()
{
  // Forward data from HW Serial to BLEUART
  //attachInterrupt(digitalPinToInterrupt(TOGGLE_PIN), toggleRecording, FALLING);
  currMillis = millis(); // start timer.
  //checkButton();

  if ((millis() - loop2Millis) >= LOOP2_TIME) {
    diffMillis = millis() - testMillis; // just to check how long it takes to return to this point
    testMillis = millis();
    if (update_zero) {
      update_zero = 0;
      imu::Quaternion quat = bno.getQuat();
      q1.a = quat.w();
      q1.b = quat.x();
      q1.c = quat.y();
      q1.d = quat.z();
    }


    imu::Quaternion quat = bno.getQuat();
    q2.a = quat.w();
    q2.b = quat.x();
    q2.c = quat.y();
    q2.d = quat.z();
    Quaternion q3 = q1.conj() * q2; // change in q1 to q2.
    //Quaternion v = Quaternion(); // create an empty vector.
    // set up the vector that we don't care about
    // note that if usb is up, it is y that we don't care about (0 1 0)
    // if it is sideways, ie battery connection up (1 0 0)
    // if it is flat and facing up (0 0 1)
    //v.a = 0; //
    //v.b = 1; // x
    //v.c = 0; // y
    //v.d = 0; // z
    Quaternion u = q3.rotate(v);
    G_angle = 180 / 3.142 * acos(u.b * v.b + u.c * v.c + u.d * v.d);
    if (isnan(G_angle)) {
      G_angle = 0; // when it is zero
    }

    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    G_acc_x = acc.x();
    G_acc_y = acc.y();
    G_acc_z = acc.z();

    
    //Serial.print("Angle: ");
    //Serial.println(G_angle);
    //Serial.println(max_tilt);
    if (G_angle < max_tilt) { // kill if more than max tilt
      if (toggle_1 == 1) {
        activate(1);
      }
      if (toggle_2 == 1) {
        activate(2);
      }
    }
    else {
      activate(0);
    }

    /*
       Old code that constrains it to a square tilt.
      if (G_y - zero_G_y > 10 || G_y - zero_G_y < -10 || G_z - zero_G_z > 10 || G_z - zero_G_z < -10) {
      activate(0);
      }
      else {
      activate(1);
      }
    */

    //Serial.print(G_y);
    //Serial.print(" ");
    //Serial.println(G_z);
    //G_altm  = (int)baro.getAltitude(); // Adafruit, but it is too slow, so we are going with Sparkfun's version.
    G_altm = (int)baro.readAltitudeFt() - G_ground_altm;
    //Serial.println(G_altm);
    read_acc();
    recordData();
    loop2Millis = millis(); // reset
    //Serial.println(diffMillis);
  }

  if ((millis() - loop3Millis) >= LOOP3_TIME) {
    //screenUpdate();
    //ble_update();
    //Serial.println("Toggle here");

    if (G_apogee < G_altm) {
      G_apogee = G_altm;
    }
    if (recording) {
      digitalToggle(GREEN_LED);
    }
    else {
      digitalWrite(GREEN_LED, LOW);
    }
    if (in_error) {
      digitalToggle(RED_LED);
    }
    else {
      digitalWrite(RED_LED, LOW);
    }
    G_batt = readVbat(); // reads battery life
    bleSystemHealth(); // reports system health over BLE
    loop3Millis = millis();
  }

  flushData();
  //read_acc();



  while (Serial.available())
  {
    // Delay to wait for enough input, since we have a limited transmission buffer
    delay(1);
    uint8_t buf[64];
    //int count = Serial.readBytes(buf, sizeof(buf));
    //bleuart.write( buf, count );
  }
  // Forward from BLEUART to HW Serial
  uint8_t rxBuf[6]={};
  int rxCount = 0;
  while ( bleuart.available() )
  {
    uint8_t ch;
    ch = (uint8_t) bleuart.read();
    //Serial.print(ch);
    if (ch == 'r') {
      startRecording();
    }
    else if (ch == 's') {
      stopRecording();
    }
    else if (ch == 'a') {
      toggle_1 = !toggle_1;
    }
    else if (ch == 'b') {
      toggle_2 = !toggle_2;
    }
    else if (ch == 'l') {
      record_lock = !record_lock;
    }
    else if (ch == 'k') {
      toggle_1 = 0;
      toggle_2 = 0;
      activate(0);
    }
    else if (ch == 'c') {
      // recalibrate 0 position
      imu::Quaternion quat = bno.getQuat();
      q1.a = quat.w();
      q1.b = quat.x();
      q1.c = quat.y();
      q1.d = quat.z();
    }
    rxBuf[rxCount] = ch;
    rxCount = rxCount + 1;
    Serial.write(ch);
  }
  if (rxBuf[0] == 't') {
    // set tilt
    max_tilt = (rxBuf[1] - '0') * 10 + (rxBuf[2] - '0');
    memset(rxBuf,0,sizeof(rxBuf));
    Serial.println(max_tilt);
  }
  
  if (rxBuf[0] == 'z') {
    if (rxBuf[1] == '1') {
      v.a = 0; //
      v.b = 1; // x
      v.c = 0; // y
      v.d = 0; // z
    }
    else if  (rxBuf[1] == '2') {
      v.a = 0; //
      v.b = 0; // x
      v.c = 1; // y
      v.d = 0; // z
    }
    else if  (rxBuf[1] == '3') {

      v.a = 0; //
      v.b = 0; // x
      v.c = 0; // y
      v.d = 1; // z
    }
    memset(rxBuf,0,sizeof(rxBuf));
  }
  if (rxBuf[0]=='h'){
    // anticipated apogee. 5 digit number
    anticipated_apogee = (rxBuf[1] - '0') * 10000 + (rxBuf[2] - '0')*1000 + (rxBuf[3] - '0')*100 + (rxBuf[4] - '0')*10 +(rxBuf[5] - '0');
    memset(rxBuf,0,sizeof(rxBuf));
    Serial.println(max_tilt);
  }

  // Request CPU to enter low-power mode until an event/interrupt occurs
  waitForEvent();
}

void bleInit() {
  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behaviour, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(true);

  Bluefruit.begin();
  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(4);
  Bluefruit.setName("Bluefruit52");
  //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
  Bluefruit.setConnectCallback(connect_callback);
  Bluefruit.setDisconnectCallback(disconnect_callback);

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Start BLE Battery Service
  blebas.begin();
  blebas.write(100);

  // Set up and start advertising
  startAdv();

  //Serial.println("Please use Adafruit's Bluefruit LE app to connect in UART mode");
  //Serial.println("Once connected, enter character(s) that you wish to send");
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
     - Enable auto advertising if disconnected
     - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
     - Timeout for fast mode is 30 seconds
     - Start(timeout) with timeout = 0 will advertise forever (until connected)

     For recommended advertising interval
     https://developer.apple.com/library/content/qa/qa1931/_index.html
  */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}


void connect_callback(uint16_t conn_handle)
{
  char central_name[32] = { 0 };
  Bluefruit.Gap.getPeerName(conn_handle, central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.println("Disconnected");
}
/*
   Init complete
   Send out init complete
*/
void init_complete() {
  char buf[64]; // buffer
  int n; // count
  for (int i = 0; i < 3; i++) { // Blink 3 times to acknowledge we are done.
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, HIGH);
    n = sprintf(buf, "Init complete %d\n", i); // Heartbeat,alt,temp
    bleuart.write(buf, n); // write to UART
    Serial.println(buf);
    delay(500);
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
    delay(500);
  }
}
/*
   Activate energetics
*/
void activate(int i) {
  if (i == 1) {
    digitalWrite(A3, HIGH);
    digitalWrite(RED_LED, HIGH);
    activate_1 = 1;
  }
  else if (i == 2) {
    digitalWrite(A5, HIGH);
    digitalWrite(RED_LED, HIGH);
    activate_2 = 1;
  }
  else {
    digitalWrite(A3, LOW);
    digitalWrite(A5, LOW);
    digitalWrite(RED_LED, LOW);
    activate_1 = 0;
    activate_2 = 0;
  }
}

/*
   LED init
*/
void pins_init() {
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(ENERGETICS_PIN_1, OUTPUT);
  pinMode(ENERGETICS_PIN_2, OUTPUT);
}

/*
   Toggles Red led
*/
void toggle_red() {
  if (red_state == 1) {
    red_state = 0;
  }
  else {
    red_state = 1;
  }
  digitalWrite(RED_LED, red_state);
}

/*
   System health, reported over BLE
*/

void bleSystemHealth() {
  char buf[64]; // buffer
  int n; // count
  n = sprintf(buf, "B: %d E: %d, R: %d, L: %d, A:%d %d, En1: %d, En2: %d, H: %d T: %4.2f\n", G_batt, in_error, recording, record_lock, activate_1, activate_2, toggle_1, toggle_2, G_altm, G_angle); //Battery level, Error status, Recording, maximum height
  bleuart.write(buf, n); // write to UART
}


/*
   Baro init
*/

void baroInit() {
  Wire.begin();
  Serial.println("Start Baro");
  baro.begin();
  //Serial.println("Here1");
  baro.setModeAltimeter();
  //Serial.println("Here2");
  baro.setOversampleRate(7);
  //Serial.println("Here3");
  baro.enableEventFlags();
  //Serial.println("Here4");
  G_ground_altm = (int)baro.readAltitudeFt();
}




/*
   ADXL377
*/
void acc_init() {
  pinMode(PIN_X, INPUT);
  pinMode(PIN_Y, INPUT);
  pinMode(PIN_Z, INPUT);
}

void read_acc() {
  int X_val = analogRead(PIN_X); // max is 3700+ ~ 3.3V
  int Y_val = analogRead(PIN_Y);
  int Z_val = analogRead(PIN_Z);
  // 3600mV/4096 * Counts = voltage
  // (voltage-1650)*(200) = acc (gs)
  // (3600/4096*1841-1650)*200/1650
  G_High_x = ((float)X_val * ACC_CALIBRATION) - 200.0;
  G_High_y = ((float)Y_val * ACC_CALIBRATION) - 200.0;
  G_High_z = ((float)Z_val * ACC_CALIBRATION) - 200.0;
  //Serial.print("X: ");Serial.print(X_f);Serial.print(" Y: ");Serial.print(Y_f);Serial.print(" Z: ");Serial.println(Z_f);
}



/*
   Flush file
*/
void flushData() {
  //Serial.print("Flush: ");Serial.print(flushTime);Serial.print(" now: ");Serial.println(millis());
  if ((millis() - flushTime) > (60000)) {
    if (recording) {
      Serial.println("Flush!");
      currentFile.flush();
    }
    flushTime = millis();
  }
}


/*
   Write to currently open file
*/
void recordData() {
  if (recording) {
    currentFile.print(millis() - startMillis); // get time
    currentFile.print(',');
    currentFile.print(G_altm);
    currentFile.print(',');
    currentFile.print(G_angle);
    currentFile.print(',');
    currentFile.print(q2.a);
    currentFile.print(',');
    currentFile.print(q2.b);
    currentFile.print(',');
    currentFile.print(q2.c);
    currentFile.print(',');
    currentFile.print(q2.d);
    currentFile.print(',');
    currentFile.print(G_acc_x);
    currentFile.print(',');
    currentFile.print(G_acc_y);
    currentFile.print(',');
    currentFile.println(G_acc_z);
  }
}

/*
   creates a file if one doesn't exist
*/

void startRecording() {
  if (recording == 1) {
    Serial.println("Already recording!");
  }
  else {
    int i = 0;
    String fileName;
    fileName = "data_";
    fileName += i;
    fileName += ".txt";
    while (SD.exists(fileName)) {
      i++;
      fileName = "data_";
      fileName += i;
      fileName += ".txt";
    }
    currentFile = SD.open(fileName, FILE_WRITE);
    flushTime = millis(); // capture time for flush..
    Serial.print("SD Card File is:"); Serial.println(fileName);
    // Generate Header
    currentFile.print("Time");
    currentFile.print(',');
    currentFile.print("Alt");
    currentFile.print(',');
    currentFile.print("Angle");
    currentFile.print(',');
    currentFile.print("qw");
    currentFile.print(',');
    currentFile.print("qx");
    currentFile.print(',');
    currentFile.print("qy");
    currentFile.print(',');
    currentFile.print("qz");
    currentFile.print(',');
    currentFile.print("High X");
    currentFile.print(',');
    currentFile.print("High Y");
    currentFile.print(',');
    currentFile.println("High Z");
    recording = 1;
    Serial.println("Starting recording!");
  }
}

void stopRecording() {
  if (recording == 0) {
    Serial.println("Already not recording!");
  }
  else {
    if (record_lock == 0) {
      // stop recording
      currentFile.close();
      recording = 0;
      digitalWrite(GREEN_LED, LOW); //turn off LED.
      Serial.println("Stopping recording!");
    }
    else {
      Serial.println("Recording is locked! Disable lock first!");
    }
  }

}

/*
    Might want to add an LED for blinkie
*/
void toggleHandler() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 400ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 400) {
    //toggle_1 = 1;
    //toggle_2 = 1;
    if (recording == 1) {
      // stop recording
      stopRecording();
    }
    else {
      startRecording();
    }
  }
  last_interrupt_time = interrupt_time;
}

void toggleHandler2() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 400ms, assume it's a bounce and ignore


  if (interrupt_time - last_interrupt_time > 400) {


    //activate(0);
    // update the center.
    update_zero = 1;

  }
  last_interrupt_time = interrupt_time;
}


void where_am_i() {
  if (launched) { // off the launch pad
    if (highest_point < G_altm) {
      highest_point = G_altm;
    }
    else { // Rocket coming down
      if (impact_detected() && G_altm < (MIN_H)) { // impact detected close to ground
        landed = 1;
      }
    }
  }
  else { // waiting on launch pad.
    if (impact_detected() && G_altm > (MIN_H)) {
      launched = 1;
    }
  }
}

int impact_detected() {
  if (G_High_x > IMPACT_G or G_High_y > IMPACT_G or G_High_z > IMPACT_G) {
    return 1;
  }
  else if (G_High_x < -IMPACT_G or G_High_y < -IMPACT_G or G_High_z < -IMPACT_G) {
    return 1;
  }
  else {
    return 0;
  }
}

/*
   Battery
*/

int readVbat(void) {
  int raw;

  // Set the analog reference to 3.0V (default = 3.6V)
  //analogReference(AR_INTERNAL_3_0);

  // Set the resolution to 12-bit (0..4095)
  //analogReadResolution(12); // Can be 8, 10, 12 or 14

  // Let the ADC settle
  //delay(1);

  // Get the raw 12-bit, 0..3600mV ADC value
  raw = analogRead(VBAT_PIN);
  uint8_t vbat_per = mvToPercent(raw * VBAT_MV_PER_LSB);
  float vbat_mv = (float)raw * VBAT_MV_PER_LSB * VBAT_DIVIDER_COMP;
  /*
    Serial.print("ADC = ");
    Serial.print(raw * VBAT_MV_PER_LSB);
    Serial.print(" mV (");
    Serial.print(raw);
    Serial.print(") ");
    Serial.print("LIPO = ");
    Serial.print(vbat_mv);
    Serial.print(" mV (");
    Serial.print(vbat_per);
    Serial.println("%)");
  */
  return vbat_per;
}

uint8_t mvToPercent(float mvolts) {
  uint8_t battery_level;

  if (mvolts >= 3000)
  {
    battery_level = 100;
  }
  else if (mvolts > 2900)
  {
    battery_level = 100 - ((3000 - mvolts) * 58) / 100;
  }
  else if (mvolts > 2740)
  {
    battery_level = 42 - ((2900 - mvolts) * 24) / 160;
  }
  else if (mvolts > 2440)
  {
    battery_level = 18 - ((2740 - mvolts) * 12) / 300;
  }
  else if (mvolts > 2100)
  {
    battery_level = 6 - ((2440 - mvolts) * 6) / 340;
  }
  else
  {
    battery_level = 0;
  }

  return battery_level;
}



/**
   Software Timer callback is invoked via a built-in FreeRTOS thread with
   minimal stack size. Therefore it should be as simple as possible. If
   a periodically heavy task is needed, please use Scheduler.startLoop() to
   create a dedicated task for it.

   More information http://www.freertos.org/RTOS-software-timer.html
*/
void blink_timer_callback(TimerHandle_t xTimerID)
{
  (void) xTimerID;
  digitalToggle(LED_RED);
  //digitalToggle(5);

  //Serial.print("Hello! ");
  //screenUpdate();
  //flushData();
  //Serial.print(G_ex);Serial.print(',');Serial.print(G_ey);Serial.print(',');Serial.println(G_ez);
}




/**
   RTOS Idle callback is automatically invoked by FreeRTOS
   when there are no active threads. E.g when loop() calls delay() and
   there is no bluetooth or hw event. This is the ideal place to handle
   background data.

   NOTE: FreeRTOS is configured as tickless idle mode. After this callback
   is executed, if there is time, freeRTOS kernel will go into low power mode.
   Therefore waitForEvent() should not be called in this callback.
   http://www.freertos.org/low-power-tickless-rtos.html

   WARNING: This function MUST NOT call any blocking FreeRTOS API
   such as delay(), xSemaphoreTake() etc ... for more information
   http://www.freertos.org/a00016.html
*/
void rtos_idle_callback(void)
{
  // Don't call any other FreeRTOS blocking API()
  // Perform background task(s) here
}

