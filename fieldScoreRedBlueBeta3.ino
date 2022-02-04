// count game pieces as they pass thru port 
// measure distance via Time Of Flight sensor VL53L1X by Adafruit 
// display via 7-segment LED backpack by Adafruit 
// @DaVinci FRC Vitruvian Bots Team 4201

// digital pins 
#define RESET_PIN 2
#define RGB_INT_PIN 4
#define TOF_XSHUT_PIN 5
#define TOF_IRQ_PIN 6
#define LED_RED_SENSED 12
#define LED_BLUE_SENSED 11
#define LED_IN_DISTANCE 10
// analog pins 
#define SENSITIVITY_PIN 3 // analog 
#define MIN_RED_PIN 2     // analog 
#define MIN_BLUE_PIN 1    // analog 

// internal logic color codes 
#define RED   1
#define GREEN 2
#define BLUE  3

// color calibration: use pot to add or subtract 500 
#define RED_BASELINE 1375   // 1875-500 <== midpoint of 1K Pot 
#define BLUE_BASELINE 2000  // 2500-500 <== 1K midpoint  
#define TRUE HIGH
#define FALSE LOW

#include "Adafruit_LEDBackpack.h"
// Adafruit VL53L1X Time Of Flight TOF distance sensor uses micro LIDAR 
#include "Adafruit_VL53L1X.h" 
// Adafruit TCS34725 RGB color sensor 
#include "Adafruit_TCS34725.h" 

/**
 * CAREFUL! - I2C ADDRESSES ARE CRITICAL!  
 * https://learn.adafruit.com/adafruit-led-backpack/changing-i2c-address 
 * You can change the address of a backpack very easily. 
 * Look on the back to find the two or three A0, A1 or A2 solder jumpers. 
 * Each one of these is used to hardcode in the address. 
 * If a jumper is shorted with solder, that sets the address. 
 * A0 sets the lowest bit with a value of 1, A1 sets the middle bit 
 * with a value of 2 and A2 sets the high bit with a value of 4. 
 * The final address is 0x70 + A2 + A1 + A0. 
 * So for example if A2 is shorted and A0 is shorted, 
 * the address is 0x70 + 4 + 1 = 0x75. 
 */
Adafruit_7segment matrix = Adafruit_7segment(); 
Adafruit_7segment matrixBlue = Adafruit_7segment(); 
Adafruit_VL53L1X distSensor = Adafruit_VL53L1X(TOF_XSHUT_PIN, TOF_IRQ_PIN);
/* Initialise with default values (int time = 2.4ms, gain = 1x) */
// Adafruit_TCS34725 colorSensor = Adafruit_TCS34725();
/* Initialise with specific int time and gain values */
Adafruit_TCS34725 colorSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

void setupLEDmatrix() {
#ifndef __AVR_ATtiny85__
  Serial.println("LED Matrix Backpack");
#endif
  matrix.begin(0x70);
  matrixBlue.begin(0x71);
}

void displayLEDmatrix(int n) {
    matrix.println(n);
    matrix.writeDisplay();
}
void displayCountRed(int n) {
    matrix.println(n);
    matrix.writeDisplay();
}
void displayCountBlue(int n) {
    matrixBlue.println(n);
    matrixBlue.writeDisplay();
}

void setupVL53L1X() {  // time of flight distance sensor 
//  Wire.begin();
  Serial.println(F("VL53L1X API Simple Ranging example\n\n"));
  if (! distSensor.begin(0x30, &Wire)) {
    Serial.print(F("Error on init of VL sensor: "));
    Serial.println(distSensor.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("VL53L1X sensor OK!"));

  Serial.print(F("Sensor ID: 0x"));
  Serial.println(distSensor.sensorID(), HEX);

  if (! distSensor.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(distSensor.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("Ranging started"));

  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  distSensor.setTimingBudget(50);
  Serial.print(F("Timing budget (ms): "));
  Serial.println(distSensor.getTimingBudget());

  /*
  vl.VL53L1X_SetDistanceThreshold(100, 300, 3, 1);
  vl.VL53L1X_SetInterruptPolarity(0);
  */ 
}

void setupTCScolorSensor() {
  if (colorSensor.begin()) {
    Serial.println("Found TCS color sensor");
  } else {
    Serial.println("No TCS34725 found. Check wiring and I2C addresses.");
    while (1);
  }
}

void setup() {
  // put your setup code here, to run once:
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_IN_DISTANCE, OUTPUT);
  pinMode(LED_RED_SENSED, OUTPUT);
  pinMode(LED_BLUE_SENSED, OUTPUT);
  pinMode(RGB_INT_PIN, OUTPUT);
  pinMode(TOF_XSHUT_PIN, OUTPUT);
  pinMode(RESET_PIN, INPUT);
  pinMode(SENSITIVITY_PIN, INPUT);
  pinMode(MIN_RED_PIN, INPUT);
  pinMode(MIN_BLUE_PIN, INPUT);

  Serial.begin(9600);
  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(10);
  }
  Wire.begin();
  setupLEDmatrix();
//  digitalWrite(TOF_XSHUT_PIN, LOW);
//  digitalWrite(RGB_INT_PIN, LOW);
  setupVL53L1X();
  setupTCScolorSensor();
}

int distance=0;
int count=0;
int gap=1;  // gap between balls 
boolean ballPresent=FALSE;
int sensitive=100;
int luxMinimum = 200;
int redMin = 700;
int redMax = 1900;
int blueMin = 2500;
int countRed=0;
int countBlue=0;
int color=0;

void reset() {
  Serial.println("RESET SCORE - RESET SCORE - RESET SCORE - RESET SCORE");
   distance=0;
   count=0;
   gap=1;
   ballPresent=FALSE;
   sensitive=100;
   luxMinimum=200;
   countRed=0;
   countBlue=0;
   color=0;
   displayCountRed( countRed );
   displayCountBlue( countBlue );
   showColorLEDs( color );
}

void checkDistance() {
  distance = distSensor.distance();
//  Serial.print(" Distance (mm)= "); Serial.println(distance);
  if( distance == -1 ) {
    Serial.println(" out of range.");
    distance=9999;
  }
  if (distance < sensitive) {
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_IN_DISTANCE, HIGH);
    Serial.print(" Distance (mm)= "); Serial.print(distance);
    if (gap == 1) {
      gap = 0;
      count++;
      Serial.print(" count= "); Serial.print(count);
    }
    Serial.print(" gap=");  Serial.println(gap);
    color = getColor();
  } else {    // nothing detected by distance sensor 
    gap=1;
    color = 0;
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(LED_IN_DISTANCE, LOW);
  }
  displayLEDmatrix(count);
  displayCountBlue(count);
}

int getColor() {
//  int color=0;
  uint16_t r, g, b, c, colorTemp, lux;
  colorSensor.getRawData(&r, &g, &b, &c);
  lux = colorSensor.calculateLux(r, g, b);
  Serial.print("distMin="); Serial.print(sensitive); Serial.print(" - ");
  Serial.print("redMax="); Serial.print(redMax); Serial.print(" - ");
  Serial.print("blueMin="); Serial.print(blueMin); Serial.print(" - ");

  Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");

  if( b > blueMin   /* 2500 */ 
    && r < redMax ) /* 1875 */ 
  {
    Serial.println(" BLUE ");
    return BLUE;
  } else { 
    Serial.println(" RED ");
    return RED;
  }
//  else {
//    Serial.println(" COLOR UNKNOWN ");
//    return 0;
//  }
//  return color;  
}

void showColorLEDs(int color) {
  if( color == RED ) {
    digitalWrite(LED_RED_SENSED, HIGH);
    digitalWrite(LED_BLUE_SENSED, LOW);
  } 
  else if( color == BLUE ) {
    digitalWrite(LED_RED_SENSED, LOW);
    digitalWrite(LED_BLUE_SENSED, HIGH);
  } else {
    digitalWrite(LED_RED_SENSED, LOW);
    digitalWrite(LED_BLUE_SENSED, LOW);    
  }
}

/**
 * MAIN CODE LOOP ########  ######## ######## ######## MAIN CODE LOOP 
 */
void loop() {
  // put your main code here, to run repeatedly:
  if( digitalRead(RESET_PIN) == HIGH ) { 
    reset(); 
  }
  sensitive = analogRead(SENSITIVITY_PIN) / 2;
//  luxMinimum = analogRead(SENSITIVITY_PIN) * 2;
  redMax = RED_BASELINE + analogRead(MIN_RED_PIN) * 2;
  blueMin = BLUE_BASELINE + analogRead(MIN_BLUE_PIN);
 
  distance = distSensor.distance();
  if( distance < sensitive ) { // ball present in range 
    ballPresent = TRUE;
    digitalWrite(LED_IN_DISTANCE, HIGH);
    color = getColor();
  } else { // no ball present in range
    if( ballPresent == TRUE ) { // trailing edge of ball 
      // ball was present in previous loop iteration 
      if( color == RED ) {
        countRed++;
        displayCountRed( countRed );
      } else { 
        countBlue++;
        displayCountBlue( countBlue );
      }
    }
    ballPresent = FALSE;
    digitalWrite(LED_IN_DISTANCE, LOW);
  }
  showColorLEDs(color);

  delay(100);
}
