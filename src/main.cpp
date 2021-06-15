#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"


// Pins for Encoders
#define ENCODER_A_M1 18
#define ENCODER_B_M1 22

#define ENCODER_A_M2 19
#define ENCODER_B_M2 23

#define ENCODER_A_M3 3
#define ENCODER_B_M3 24

// Pins for Buttons
#define BUTTON1_M1 53
#define BUTTON2_M1 51

#define BUTTON1_M2 49
#define BUTTON2_M2 47

#define BUTTON1_M3 45
#define BUTTON2_M3 43

#define BUTTON1_M4 41
#define BUTTON2_M4 39

// Pins for Motor Driver (must be PWM capable)
#define IN1_M1 7
#define IN2_M1 11
#define IN1_M2 6
#define IN2_M2 12
#define IN1_M3 5
#define IN2_M3 13

// Pins for G2 Motor Driver
#define PWM_PIN 2
#define DIR_PIN 1

// Pins for Potentiometers
#define POT1 A15
#define POT2 A11
#define POT3 A7
#define POT4 A3

volatile long encoder_ticks, encoder_ticks1, encoder_ticks2, encoder_ticks3, encoder_ticks4 = 0;
int aState1, aState2, aState3, aState4; 
int aLastState1, aLastState2, aLastState3, aLastState4 = 0;

int min_pot = 0; 
int max_pot = 1023;

const int cpr = 5; // encoder counts per revolution of motor shaft
const float gear_ratio = 125;



float quatI, quatJ, quatK, quatReal, quatRadianAccuracy = 0;
float actual_angle;


void setSpeedDirection(byte motor_number, byte b1, byte b2, byte pwm_val) {
  byte forward_pin; byte backward_pin;
  switch (motor_number) {
    case 1: 
      forward_pin = IN1_M1;
      backward_pin = IN2_M1;
      break;
    case 2:
      forward_pin = IN1_M2;
      backward_pin = IN2_M2;
      break;  
    case 3: 
      forward_pin = IN1_M3;
      backward_pin = IN2_M3;
      break;
  }
  if (b1 == HIGH) {
  if (b2 == LOW) {
    analogWrite(forward_pin, pwm_val);
    } else {
    analogWrite(forward_pin, 0);
    analogWrite(backward_pin,0);
    }
 }
 if (b1 == LOW) {
  if (b2 == LOW) {
    analogWrite(forward_pin, 0);
    analogWrite(backward_pin,0);
    } else {
    analogWrite(backward_pin, pwm_val);
    }
 } 
}

int potConversion(byte analog_pin) {
  int pot_val;
  switch (analog_pin) {
    case 1: pot_val = analogRead(POT1); break;
    case 2: pot_val = analogRead(POT2); break;
    case 3: pot_val = analogRead(POT3); break;
    case 4: pot_val = analogRead(POT4); break;
  }
  pot_val = constrain(pot_val, min_pot, max_pot);
  byte pwm_val = map(pot_val, min_pot, max_pot, 255, 0);
  return pwm_val;
}

void encoder_event1() {
 aState1 = digitalRead(ENCODER_A_M1); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   if (aState1 != aLastState1){     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(ENCODER_B_M1) != aState1) {encoder_ticks1 ++;} 
     else {encoder_ticks1 --;}}
     //aLastState1 = aState1;
     }
void encoder_event2() {
 aState2 = digitalRead(ENCODER_A_M2);
   if (aState2 != aLastState2){     
     if (digitalRead(ENCODER_B_M2) != aState2) {encoder_ticks2 ++;} 
     else {encoder_ticks2 --;}}
     //aLastState2 = aState2;
     }
void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(BUTTON1_M1, INPUT); pinMode(BUTTON2_M1, INPUT);
  pinMode(BUTTON1_M2, INPUT); pinMode(BUTTON2_M2, INPUT);
  pinMode(BUTTON1_M3, INPUT); pinMode(BUTTON2_M3, INPUT);
  pinMode(BUTTON1_M4, INPUT); pinMode(BUTTON2_M4, INPUT);
  pinMode(IN1_M1, OUTPUT); pinMode(IN2_M1, OUTPUT);
  pinMode(IN1_M2, OUTPUT); pinMode(IN2_M2, OUTPUT);
  pinMode(IN1_M3, OUTPUT); pinMode(IN2_M3, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  //pinMode(ENCODER_B_M1, INPUT); pinMode(ENCODER_B_M2, INPUT);
  //pinMode(ENCODER_B_M3, OUTPUT); pinMode(ENCODER_B_M4, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_M1), encoder_event1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_M2), encoder_event2, CHANGE);




  Serial.println("Starting Program...");
}

void loop() {
 setSpeedDirection(1, digitalRead(BUTTON1_M1), digitalRead(BUTTON2_M1), potConversion(1));
 setSpeedDirection(2, digitalRead(BUTTON1_M2), digitalRead(BUTTON2_M2), potConversion(2));
 setSpeedDirection(3, digitalRead(BUTTON1_M3), digitalRead(BUTTON2_M3), potConversion(3));
 /*
   if (myIMU.dataAvailable() == true) {
    quatI = myIMU.getQuatI();
    quatJ = myIMU.getQuatJ();
    quatK = myIMU.getQuatK();
    quatReal = myIMU.getQuatReal();
    quatRadianAccuracy = myIMU.getQuatRadianAccuracy();
    actual_angle = asin(2 * (quatReal * quatJ - quatI * quatK)) * 180 / M_PI;
  }
  Serial.print("Rotations: ");
  Serial.print(encoder_ticks1/gear_ratio/cpr);
  Serial.print(", Angle (deg):");
  Serial.print(actual_angle);
  Serial.print(", QuatI: ");
  Serial.print(quatI);
  Serial.print(", QuatJ: ");
  Serial.print(quatJ);
  Serial.print(", QuatK: ");
  Serial.print(quatK);
  Serial.print(", QuatReal: ");
  Serial.println(quatReal);
  */
/*
 Serial.print("Encoder Ticks: M1: "); Serial.print(encoder_ticks1);
 Serial.print(", M2: "); Serial.print(encoder_ticks2);
 Serial.print(", M3: "); Serial.print(encoder_ticks3);
 Serial.print(", M4: "); Serial.println(encoder_ticks4);
 */
 Serial.print(digitalRead(BUTTON1_M1)); Serial.print(digitalRead(BUTTON2_M1));
 Serial.print(digitalRead(BUTTON1_M2)); Serial.print(digitalRead(BUTTON2_M2));
 Serial.print(digitalRead(BUTTON1_M3)); Serial.print(digitalRead(BUTTON2_M3));
 Serial.print(digitalRead(BUTTON1_M4)); Serial.println(digitalRead(BUTTON2_M4));
 
 /*Serial.print(analogRead(POT1)); Serial.print("    ");
 Serial.print(analogRead(POT2)); Serial.print("    ");
 Serial.print(analogRead(POT3)); Serial.print("    ");
 Serial.println(analogRead(POT4));
 */
 //delay(250);
 }





