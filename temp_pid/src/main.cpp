#include "max6675.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>
int thermoDO = 4;
int thermoCS = 5;
int thermoCLK = 6;
int potPin = A0;    // select the input pin for the potentiometer
int ovenPin = 10;// LED output pin

#define BUTTON_PIN 7
#define RELAY_PIN 8
#define POTENTIOMETER_PIN A1
float delayTime = 0; // Giá trị thời gian delay mặc định\


// Tuning parameters
float Kp = 10; // Proportional gain
float Ki = 10; // Integral gain
float Kd = 0; // Differential gain
// Record the set point as well as the controller input(y) and output(u)
double Setpoint, y, u;
// Create a controller that is linked to the specified Input, Ouput and Setpoint
PID myPID(&y, &u, &Setpoint, Kp, Ki, Kd, DIRECT);
const int sampleRate = 1; // Time interval of the PID control

LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

void updateLCD();

void setup() {
  Serial.begin(9600);
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Cấu hình chân nút nhấn với điện trở nội kéo lên (pull-up)
  pinMode(RELAY_PIN, OUTPUT);
  lcd.init();                      // initialize the lcd
  lcd.init();
  lcd.backlight();
  updateLCD(); // Cập nhật giá trị thời gian mặc định lên màn hình
  myPID.SetMode(AUTOMATIC);     // Turn on the PID control
  myPID.SetSampleTime(sampleRate); // Assign the sample rate of the control
}

void loop() {
  Setpoint = map(analogRead(potPin), 0, 1023, 0, 255); // read and scale the set point
  y = thermocouple.readCelsius();
  myPID.Compute(); // Calculates the PID output at a specified sample time
  analogWrite(ovenPin, u); // Send output to oven
  static long preTick = 0;
  if (millis() - preTick >= 1000)
  {
    preTick = millis();
    Serial.print("C = ");
    Serial.println(y);

    Serial.print("Output:");
    Serial.println(u);
  }

  int buttonState = digitalRead(BUTTON_PIN);
  int potValue = analogRead(POTENTIOMETER_PIN);
  delayTime = map(potValue, 0, 1021, 1000, 10000);
  updateLCD();
  if (buttonState == LOW) {
    digitalWrite(RELAY_PIN, HIGH);
    unsigned long startTime = millis();
    while (millis() - startTime < delayTime) {

    }
    digitalWrite(RELAY_PIN, LOW);
  }

  // The tuning parameters can be retrieved by the Arduino from the serial monitor: eg: 0,0.5,0 with Ki set to 0.5.
  // Commas are ignored by the Serial.parseFloat() command
  if (Serial.available() > 0) {
    for (int i = 0; i < 4; i++) {
      switch (i) {
        case 0:
          Kp = Serial.parseFloat();
          break;
        case 1:
          Ki = Serial.parseFloat();
          break;
        case 2:
          Kd = Serial.parseFloat();
          break;
        case 3:
          for (int j = Serial.available(); j == 0; j--) {
            Serial.read();
          }
          break;
      }
    }
    Serial.print(" Kp,Ki,Kd = "); // Display the new parameters
    Serial.print(Kp);
    Serial.print(",");
    Serial.print(Ki);
    Serial.print(",");
    Serial.print(Kd);
    Serial.println();
    myPID.SetTunings(Kp, Ki, Kd); // Set the tuning of the PID loop
  }
}

void updateLCD() {

  //Display on LCD

  lcd.setCursor(0, 0);
  lcd.print("Temp:");
  lcd.setCursor(5, 0);
  lcd.print(y);
  lcd.setCursor(10, 0);
  lcd.write(byte(0xDF)); // Print the degree symbol
  lcd.print("C");
  /////////////////////////
  lcd.setCursor(0, 1);
  lcd.print("Settemp:");
  lcd.print(Setpoint);
  lcd.write(0xdf); // to display °
  lcd.print("C");
  delay(200);
  /////////////////////////////////
  lcd.clear();
  lcd.setCursor(13, 0);
  lcd.print("T: ");
  lcd.setCursor(15, 0);
  lcd.print(delayTime / 1000);
  lcd.print(" s");
  delay(50);
}
