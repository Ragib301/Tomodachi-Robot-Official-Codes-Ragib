#define USE_ARDUINO_INTERRUPTS false  // Set-up low-level interrupts for most acurate BPM math

#include "DHT.h"
#include <Servo.h>
#include "Wire.h"
#include <TinyGPS++.h>
#include <TimerFreeTone.h>
#include <PulseSensorPlayground.h>
#include <NewPing.h>
#include <LiquidCrystal_I2C.h>

// Temperature Object & Variable Declare
#define DHTTYPE DHT11
DHT temp_dht(A4, DHTTYPE);
int BodyTemp;

// Hearbeat Pulse Sensor
#define PulseWire A3       // 'S' Signal pin connected to A3
const int PULSE_BLINK = 13;    // Pin 13 is the on-board LED
const int PULSE_FADE = 5;
const int THRESHOLD = 550;   // Adjust this number to avoid noise when idle
byte samplesUntilReport;
const byte SAMPLES_PER_SERIAL_SAMPLE = 10;

PulseSensorPlayground pulseSensor;  // Creates an object
int heartbeatBPM;

// LCD Monitor Declare
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Servo Object Declare
Servo leftHand, rightHand, maskServo, pill_dispenser;
#define Position 100
int pos = 0;
#define servo_delay 20

// Room Detection Ultrasonic Module
#define TRIGGER_PIN  A6  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     A7  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 350 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
int distance;

// Ground Level Detection Ultrasonic Module
NewPing level_sonar(A8, A9, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
#define threshold_level 7
uint32_t m1;
boolean ground_loop = true;
int ground_level, stone_time = 1000;

// L298N Motor Driver
#define IN1 26
#define IN2 25
#define IN3 22
#define IN4 23
#define SpeedPin 7
#define Motor_Speed 200

// Motor Braking Time
#define brake_time 0
#define Ninety_Degree_delay 700

// Object & Building Detection Sensor
#define Object_Detector A2
#define room_distance 25
int room_number, count = 0;
#define room_avoid_delay 500
#define home_point_delay 3500
#define Kitchen_number 3

// Neo-6M GPS Module
TinyGPSPlus gps;
#define gpsSerial Serial2
float lati, longi;
String googlemapURL = "https://www.google.com/maps/@";

// Bluetooth Module (For Mobile)
#define BTSerial Serial1
char command, mode, heavy_mode;
String Emergency_Messeage;

// Bluetooth Module (Slave)
#define BedSerial Serial3
String sleep_state;

// Push Button Pin
#define Button A0
#define LaserGun 8

// PIR Sensor
#define Human_Detector A5

// Scream detection sensor
#define Sound_Sensor A1

// Buzzer, Speaker & SD Card Module
#define SpeakerPin 11
#define BuzzerPin 2
boolean buzz_state = true;

// Define color sensor pins
#define S0 28
#define S1 29
#define S2 30
#define S3 31
#define sensorOut 32

// Stabilization Delay
#define stab_delay 200

// Calibration Values
int redMin = 19;      // Red minimum value
int redMax = 230;     // Red maximum value

int greenMin = 18;    // Green minimum value
int greenMax = 220;   // Green maximum value

int blueMin = 14;     // Blue minimum value
int blueMax = 163;    // Blue maximum value

// Variables for Color Pulse Width Measurements
int redPW   = 0;
int greenPW = 0;
int bluePW  = 0;

// Variables for final Color values
int redValue, greenValue, blueValue;

#define Baund 9600

void setup() {
  Serial.begin(Baund);
  BTSerial.begin(Baund);
  gpsSerial.begin(Baund);
  BedSerial.begin(38400);

  // DHT11 Sensor Begin
  temp_dht.begin();

  // Configure the PulseSensor object, by assigning our variables to it
  pulseSensor.analogInput(PulseWire);
  pulseSensor.setThreshold(THRESHOLD);
  pulseSensor.blinkOnPulse(PULSE_BLINK);
  pulseSensor.fadeOnPulse(PULSE_FADE);
  pulseSensor.setSerial(Serial);
  pulseSensor.setOutputType(SERIAL_PLOTTER);

  // Skip the first SAMPLES_PER_SERIAL_SAMPLE in the loop().
  samplesUntilReport = SAMPLES_PER_SERIAL_SAMPLE;

  // Initialize lcd screen
  lcd.init();
  // Turn on the backlight
  lcd.backlight();

  rightHand.attach(3);
  leftHand.attach(4);
  maskServo.attach(6);
  pill_dispenser.attach(5);

  rightHand.write(Position);
  leftHand.write(Position - 20);
  maskServo.write(Position);
  pill_dispenser.write(Position);

  pinMode(Button, INPUT_PULLUP);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(SpeedPin, OUTPUT);

  pinMode(Object_Detector, INPUT);
  pinMode(Human_Detector, INPUT);
  pinMode(BuzzerPin, OUTPUT);
  pinMode(SpeakerPin, OUTPUT);
  pinMode(LaserGun, OUTPUT);

  // Set S0 - S3 as outputs
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  // Set Sensor output as input
  pinMode(sensorOut, INPUT);

  // Setting Clear Filter
  digitalWrite(S2, HIGH);
  digitalWrite(S3, LOW);

  // Set Pulse Width scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  Serial.println("Ready!");
  analogWrite(SpeedPin, Motor_Speed);
}

void(*resetFunc) (void) = 0; // Declare reset function @ address 0

void loop() {
  GPS_System();
  lcd.setCursor(2, 0);
  lcd.print("|   TOMODACHI  |");
  lcd.setCursor(2, 1);
  lcd.print("|A SOCIAL ROBOT|");

  /* All Function Loops */
  Button_Performer();
  Heavy_Object_Delivery_Function();
  Covid_19_Function();
}

void Button_Performer() {
  int ButtonPress = button_read(Button, LaserGun);

  if (ButtonPress != 0) {
    rightHand.write(0);
    if (ButtonPress > 4) emergency_button();
    else if (ButtonPress == 3 ) {
      rightHand.write(Position);
      digitalWrite(LaserGun, LOW);
      manual_controller();
    }
  }
}

int button_read(int sw, int light) {
  int count = 0;
p: int t = 0;

  if (digitalRead(sw) == LOW) {
    digitalWrite(light, HIGH);
    while (digitalRead(sw) == LOW) {
      delay(1); t++;
    }
    digitalWrite(light, LOW);
    if (t > 10) {
      t = 0; count++;
      while (digitalRead(sw) == HIGH) {
        delay(1); t++;
        if (t > 1000) return count;
      }
      goto p;
    }
  } return count;
}

void Heavy_Object_Delivery_Function() {
  while (digitalRead(Object_Detector) == LOW) {
    rightHand.write(0);
    leftHand.write(180);

    BuzzerTone();
    count = 0;

    lcd.clear();
    lcd.setCursor(2, 0);
    lcd.print("Object Placed");
    lcd.setCursor(2, 1);
    lcd.print("Successfully!");
    delay(2000);

    while (BTSerial.available()) {
      lcd.clear();
      lcd.setCursor(2, 0);
      lcd.print("Object Delivery");
      lcd.setCursor(2, 1);
      lcd.print("is in Progress!");
      delay(100);

      mode = BTSerial.read();
      while (mode == 'M') manual_controller();
      while (mode == 'A') auto_delivery();
    }
  }
}

int readping() {
  delay(50);
  int dis = sonar.ping_cm();
  if (dis == 0) dis = 255;
  return dis;
}

void auto_delivery() {
  while (BTSerial.available()) {
    room_number = BTSerial.parseInt();

    if (room_number > 0) {
      BuzzerTone();
      while (true) {
        building_detect();
      }
    }
  }
}

void building_detect() {
  forward();
  distance = readping();
  delay(200);

  if (distance <= room_distance) {
    count++;
    brake(); delay(500);

    if (room_number == count) {
      BuzzerTone();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Robot has reached");
      lcd.setCursor(0, 1);
      lcd.print("at the Destination!");
      delay(500);

      left();  delay(Ninety_Degree_delay);
      stopme(); delay(brake_time);

      delay(100);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Object Delivery");
      lcd.setCursor(1, 1);
      lcd.print("is Completed!");
      exit(0);
    }
    else {
      forward();
      delay(room_avoid_delay);
    }
  }
}

void manual_controller() {
  BuzzerTone();

  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print(" Bluetooth");
  lcd.setCursor(2, 1);
  lcd.print("Controlling!");

  while (true) {
    while (BTSerial.available()) {
      command = BTSerial.read();
      switch (command) {

        case 'F':         // Forward
          forward();
          break;

        case 'B':         // Backward
          backward();
          break;

        case 'L':         // Go Left
          left();
          break;

        case 'G':
          front_left();
          break;

        case 'H':
          back_left();
          break;

        case 'R':         // Go Right
          right();
          break;

        case 'I':
          front_right();
          break;

        case 'J':
          back_right();
          break;

        case 'V':         // Buzzer HIGH
          digitalWrite(BuzzerPin, HIGH);
          break;

        case 'v':         // Buzzer LOW
          digitalWrite(BuzzerPin, LOW);
          break;

        case 'S':         // Stop
          stopme();
          break;

        case 'D':
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Object Delivery");
          lcd.setCursor(1, 1);
          lcd.print("is Completed!");
          exit(0);

        default:         // Garbage Data Stop
          stopme();
      }
    }
    if (ground_loop == true) level_detection();
  }
}

void level_detection() {
  ground_level = readping_level();
  if (ground_level > threshold_level) {
    stopme();
    BuzzerTone();

    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print(" Low level ground");
    lcd.setCursor(1, 1);
    lcd.print("in front! Warning!");
    delay(1000);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Filling low level");
    lcd.setCursor(0, 1);
    lcd.print("ground with stones.");
    delay(500);

    BuzzerTone();
    pill_dispenser.write(0);
    delay(10000);

    if (ground_level <= threshold_level) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Ground Filling");
      lcd.setCursor(0, 1);
      lcd.print("is completed!");
      delay(300); resetFunc();
    }
    else {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Ground Filling");
      lcd.setCursor(0, 1);
      lcd.print("isn't completed!");

      Emergency_Messeage = "A Pathole is detected in this loaction! Please Take necessary action.";
      send_SMS();
    }
  }
}

int readping_level() {
  delay(50);
  int level = level_sonar.ping_cm();
  if (level == 0) level = 255;
  return level;
}

void forward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void backward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void left() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void front_left() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void back_left() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void right() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void front_right() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void back_right() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopme() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void brake() {
  backward(); delay(brake_time);
  stopme(); delay(brake_time);
}

void BuzzerTone() {
  digitalWrite(BuzzerPin, HIGH);
  delay(900);
  digitalWrite(BuzzerPin, LOW);
  delay(500);
}

void GPS_System() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isValid()) {
    lati  = gps.location.lat();
    longi = gps.location.lng();
  }
  else {
    lati  = 23.770946;
    longi = 90.361998;
  }
}

void send_SMS() {
  stopme();

  /* Text content */
  BTSerial.println(Emergency_Messeage);
  BTSerial.print("\n");
  BTSerial.println("Google Map Link : ");
  BTSerial.print(googlemapURL);
  BTSerial.print(lati, 6);
  BTSerial.print(",");
  BTSerial.println(longi, 6);
  BTSerial.print("\n");
  BTSerial.println(char(26));

  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("Emergency SMS");
  lcd.setCursor(3, 1);
  lcd.print("has sent. OKAY  ");

  while (buzz_state == true) BuzzerTone();
}

void emergency_button() {
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Danger Function");
  lcd.setCursor(1, 1);
  lcd.print("is Activated!");
  delay(1000);

  Emergency_Messeage = "Anybody while walking on roads maybe in danger! Please Take necessary action.";
  send_SMS();
}

void Covid_19_Function() {
  while (BedSerial.available()) {
    sleep_state = BedSerial.readString();

    if (sleep_state == "1") {
      BuzzerTone();

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Covid-19 Checking");
      lcd.setCursor(4, 1);
      lcd.print("Fucntion!");
      delay(500);

      forward();
      delay(600);
      stopme();
      delay(1000);

      lcd.clear();
      leftHand.write(180);
      delay(1000);
      lcd.setCursor(0, 0);
      lcd.print("Measuring Temp...");
      delay(2000);
      BodyTemp = temp_dht.readTemperature(true) + 9;
      delay(100);
      lcd.setCursor(0, 1);
      lcd.print("Body Temp: " + String(BodyTemp) + "*F");
      delay(2000);

      if (BodyTemp >= 100) {
        BuzzerTone();
        lcd.setCursor(0, 2);
        lcd.print("Temp. is very HIGH!");
        lcd.setCursor(0, 3);
        lcd.print("Door willn't open!");
        BedSerial.write('0');
        exit(0);
      }
      else {
        BuzzerTone();
        lcd.setCursor(0, 2);
        lcd.print("Temp. is normal!");
        lcd.setCursor(0, 3);
        lcd.print("Door is opening...");
        BedSerial.write('A');
        exit(0);
      }
    }
  }
}
