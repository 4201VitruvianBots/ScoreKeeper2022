// count game pieces as they pass thru port
// measure distance via Time Of Flight sensor VL53L1X by Adafruit
// display via 7-segment LED backpack by Adafruit
// Finally, send score to SPI Master LED via SPI (see ISR) 
// @DaVinci FRC Vitruvian Bots Team 4201

// GLOBAL VARIABLES FOR CALIBRATION OF 4 MODULES 
int blueDelta=150;  // module 1
//int blueDelta=204;  // module 2
//int blueDelta=203;  // module 3
//int blueDelta=132;  // module 4


// digital pins
#define RESET_PIN 2
//#define RGB_INT_PIN 4
#define TOF_XSHUT_PIN 5
#define TOF_IRQ_PIN 6
#define LED_RED_SENSED 7
#define LED_BLUE_SENSED 8
#define LED_IN_DISTANCE 9
// analog pins
#define SENSITIVITY_PIN 3 // analog
#define MIN_RED_PIN 2     // analog
#define BLUE_DELTA_PIN 1    // analog
// internal logic color codes
#define RED   1
#define GREEN 2
#define BLUE  3
// color calibration: use pot to add or subtract 500
#define RED_BASELINE 1375   // 1875-500 <== midpoint of 1K Pot
#define BLUE_BASELINE 2000  // 2500-500 <== 1K midpoint
#define TRUE HIGH
#define FALSE LOW

uint8_t countRed=0;
uint8_t countBlue=0;
int color=0;

// SPI section follows... 

// SPDR = SPI Data Register for I/O  

/* Vitruvian Bots Team 4201 Score Keeper Slave Simulation 
   Example of SPI transfer between (1) Master controller and (4) Slave
   remote peripherals. The Master synchronizes the Slave to zero their 
   counters then collects their indiviual count of Blue and Red balls
   scored and sums them for display. 
   
   The Arduino Slave SPI SS pin 10 is tied to pin 2, as a hardwire 
   driven interupt 0, that is how the interupt is defined in this case.  
   
   K.Urashima
 */
 
#include <SPI.h>

const byte initSlave0 = 0b11111111;
const byte initSlave1 = 0b01111111;
const byte sendBlue = 0b11110000;
const byte sendRed =  0b01110000;
const byte caseZero = 0b00000000;
volatile byte commandFromMaster = 0;
volatile boolean startCount = false;

// minimize rework by remove dependency on pin 2 hard interrupt 
void ss_falling ()
{
  commandFromMaster = 0;
} // end of interrupt service routine (ISR) ss_falling

void setupSPI() {
  pinMode(MISO,OUTPUT);              //Sets MISO as OUTPUT (Have to Send data to Master IN 
  SPCR |= _BV(SPE);                  //Turn on SPI in Slave Mode
  SPCR |= _BV(SPIE);                 //Interuupt ON is set for SPI commnucation
  attachInterrupt (0, ss_falling, FALLING); // interrupt for SS falling edge
  Serial.print("SPI SS pin=");  Serial.println(SS);
}                                    //End setupSPI

ISR (SPI_STC_vect)                   // Interrupt routine function 
{
  commandFromMaster = SPDR;   // command from SPI Master 
  switch(commandFromMaster){
      case initSlave0:{
        SPDR = commandFromMaster;
//        Serial.println("SPI commandFromMaster initSlave0");
        break;   
        }
      case initSlave1:{
        countRed=0;
        countBlue=0;
        SPDR = 0;
        startCount = true; 
//        Serial.println("SPI command initSlave1 reset score counters");
        break;
      }
      case sendBlue:{
        SPDR = countBlue;
//        Serial.println("SPI command = sendBlue");
        break;
        }
      case sendRed:{
        SPDR = countRed;
//        Serial.println("SPI commmand = sendRed");
        break;
        }
      case caseZero:{        
        SPDR = 0;
        break;
        }

    }  //end of switch
}      //end of SPI ISR interrupt handler
// end of SPI section 

#include "Adafruit_LEDBackpack.h"
// Adafruit VL53L1X Time Of Flight TOF distance sensor uses micro LIDAR
#include "Adafruit_VL53L1X.h"
// Adafruit TCS34725 RGB color sensor
#include "Adafruit_TCS34725.h"
/**
 * NOTE I2C ADDRESSES ARE CRITICAL!
 * https://learn.adafruit.com/adafruit-led-backpack/changing-i2c-address
 * You can change the address of a backpack very easily.
 * Look on the back for A0 A1 A2 solder jumpers.
 * If a jumper is shorted with solder, that sets the address.
 * A0 sets the lowest bit with a value of 1
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
  delay(1);
  displayCountRed(0);
  displayCountBlue(0);
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
  Serial.println(F("init VL53L1X API ToF Ranging Distance Sensor\n"));
  if (! distSensor.begin(0x30, &Wire)) {
    Serial.print(F("Error on init of VL sensor: "));
    Serial.println(distSensor.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("VL53L1X distance sensor OK!"));
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
    Serial.println("No color sensor TCS34725 found. Check wiring and I2C addresses.");
    while (1);
  }
}

// ######## ######## ######## ######## ######## ######## SETUP 
void setup() {
  // put your setup code here, to run once:
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_IN_DISTANCE, OUTPUT);
  pinMode(LED_RED_SENSED, OUTPUT);
  pinMode(LED_BLUE_SENSED, OUTPUT);
//  pinMode(RGB_INT_PIN, OUTPUT);
//  pinMode(TOF_XSHUT_PIN, OUTPUT);
  pinMode(RESET_PIN, INPUT);
  pinMode(SENSITIVITY_PIN, INPUT);
  pinMode(MIN_RED_PIN, INPUT);
  pinMode(BLUE_DELTA_PIN, INPUT);
  Serial.begin(115200);
  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(10);
  }
  setupSPI();
  Wire.begin();
  setupLEDmatrix();
//  digitalWrite(TOF_XSHUT_PIN, LOW);
//  digitalWrite(RGB_INT_PIN, LOW);
  setupVL53L1X();
  setupTCScolorSensor();
}

// ######## ######## ######## ######## ######## ######## GLOBAL VARIABLES 
int distance=0;
int count=0;
int gap=1;  // gap between balls
boolean ballPresent=FALSE;
int sensitive=100;
int luxMinimum = 200;
int redMin = 700;
int redMax = 1900;        //possible solution, increase redmax
int blueMin = 2500;

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
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(LED_IN_DISTANCE, LOW);
  }
  displayCountRed(count);
  displayCountBlue(count);
}

int getColor() {
//  int color=0;
  uint16_t r, g, b, c, colorTemp, lux;
  colorSensor.getRawData(&r, &g, &b, &c);
//  lux = colorSensor.calculateLux(r, g, b);
  Serial.print("distMin="); Serial.print(sensitive); Serial.print(" - ");
  Serial.print("bluedelta="); Serial.print(blueDelta); Serial.print(" - ");
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  //if( b > blueMin   /* 2500 */
    //&& r < redMax ) /* 1875 */
  if( r < b + blueDelta )   //muhammad thinks possible solltion 2
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
  //sensitive = analogRead(SENSITIVITY_PIN) / 2;
  sensitive = 105;
  //redMax = RED_BASELINE + analogRead(MIN_RED_PIN) * 2;
  redMax = 330;
//  if( startCount == false ) Serial.println("SPI init command not received");
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
  delay(1);
}
