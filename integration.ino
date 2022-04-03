#include <Servo.h>

Servo myservo;
#define servoPin 0

int angle = 90;
int claw_angle = 0;

int IN1=7;
int IN2=5;
int IN3=4;
int IN4=2;
int ENA=6;
int ENB=3;

void setup() {
  myservo.attach(servoPin);
  
  for (int i = 5; i <11; i ++) {
    pinMode(i, OUTPUT);  
  }
}

void loop() {
  // rotate CW
 digitalWrite(IN1,HIGH);
 digitalWrite(IN2,HIGH);
analogWrite(ENA,200);
 digitalWrite(IN3,HIGH);
 digitalWrite(IN4,HIGH);
analogWrite(ENB,200);
 delay(2000);


 myservo.write(-90);
  delay(1000);

  // Sweep from 0 to 180 degrees:
  for (angle = -180; angle <= 90; angle += 1) {
    myservo.write(angle);
    delay(5);
  }

  // Back from 180 to 0 degrees:
  for (angle = 90; angle >= -180; angle -= 1) {
    myservo.write(angle);
    delay(5);
  }
  
 // pause for 2S
 analogWrite(ENA,0);
analogWrite(ENB,0);
  delay(2000);


  digitalWrite(IN1,LOW);
   digitalWrite(IN2,HIGH);
  analogWrite(ENA,200);
   digitalWrite(IN3,LOW);
   digitalWrite(IN4,HIGH);
  analogWrite(ENB,200);
 delay(2000);
 // pause for 2S
 analogWrite(ENA,0);
analogWrite(ENB,0);
 delay(2000);
}
