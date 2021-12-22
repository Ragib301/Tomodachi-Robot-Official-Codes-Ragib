/* Covid-19 Function's Door Part Code */

#include <Servo.h>
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(10, 11); // RX, TX
String state;
Servo myServo;
#define Position 70
#define IR_Sensor 8

void setup() {
  BTSerial.begin(38400);
  myServo.attach(9);
  myServo.write(175);
  pinMode(IR_Sensor, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  if (digitalRead(IR_Sensor) == LOW) {
    BTSerial.write('1');
    delay(1000);
  }
  if (BTSerial.available()) {
    state = BTSerial.readString();

    if (state == "A") {
      digitalWrite(LED_BUILTIN, HIGH);
      myServo.write(Position);
      delay(2000);
      myServo.write(180);
      exit(0);
    }
  }
}
