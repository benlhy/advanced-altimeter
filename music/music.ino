#define BUZZER A5

void setup() {
  // put your setup code here, to run once:
  pinMode(A5,OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i = 0;i<100;i++){
    analogWrite(BUZZER,200);
    delay(1);
    analogWrite(BUZZER,0);
    delay(1);
  }
  analogWrite(BUZZER,0);
  delay(100);

}
