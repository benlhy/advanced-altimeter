#define PIN_X A0
#define PIN_Y A1
#define PIN_Z A2

#define VOLTAGE_M 1587 // measured
#define ACC_CALIBRATION 3600/4096*200/VOLTAGE_M // 12 bits, 3.6V reference

float G_High_x;
float G_High_y;
float G_High_z;


void setup() {
  Serial.begin(9600);
  analogReadResolution(12); // setup resolution for sensor
  acc_init();
  

}

void loop() {
  // put your main code here, to run repeatedly:
  read_acc();
  delay(100);

}

/*
 * ADXL377
 */
 void acc_init(){
  pinMode(PIN_X,INPUT);
  pinMode(PIN_Y,INPUT);
  pinMode(PIN_Z,INPUT);
 }

 void read_acc(){
  int X_val = analogRead(PIN_X); // max is 3700+ ~ 3.3V
  int Y_val = analogRead(PIN_Y);
  int Z_val = analogRead(PIN_Z);
  // 3600mV/4096 * Counts = voltage
  // (voltage-1650)*(200) = acc (gs)
  // (3600/4096*1841-1650)*200/1650
  G_High_x = ((float)X_val*ACC_CALIBRATION)-200.0; 
  G_High_y = ((float)Y_val*ACC_CALIBRATION)-200.0;
  G_High_z = ((float)Z_val*ACC_CALIBRATION)-200.0;
  Serial.print("X: ");Serial.print(G_High_x);Serial.print(" Y: ");Serial.print(G_High_y);Serial.print(" Z: ");Serial.println(G_High_z);
 }
