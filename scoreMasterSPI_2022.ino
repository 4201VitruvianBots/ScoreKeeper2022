/**
 * Scorekeeper Master 
 * Collect scores from 4 portals via Arduino slave devices
 * Increment score on LARGE LED 
 */
String myPurpose="SPI Master collects scores from all slave score keepers";
#define MAIN_RED_PIN A3
#define MAIN_BLUE_PIN A6 
#define I2C_SDA_PIN   A4
#define I2C_SCL_PIN   A5 
#define RESET_RED_PIN A2
#define RESET_BLUE_PIN A7 
#define RED 1
#define BLUE 2
#define mainScoreFlashTime 100  // how quickly can main score LED respond? 
#define mainScoreRestTime  300
#define WILD_LEAP 20

/* Vitruvian Bots Team 4201 Score Keeper
   Example of SPI transfer between (1) Master controller and (4) Slave
   remote peripherals. The Master synchronize the Slave to zero their 
   counters then collects their indiviual count of Blue and Red balls
   scored and sums them for display.

   The Arduino Slave SPI SS pin 10 is tied to pin 2, as a hardwire 
   driven interupt 0, that is how the interupt is defined in this case.  
   
   Not sure the library allows remapping the Slave SS from pin 10 to another pin. The
   Master SS pins are just Digital Ouputs. K.Urashima
 */
#include <SPI.h>

int numSS = 2;  //Sets number of Nanos wired up, handles up to 4
int SSpin[] = {10,9};  // orig {10, 8, 7, 6};
const byte initSlave0 = 0b11111111;
const byte initSlave1 = 0b01111111;
const byte getBlue = 0b11110000;
const byte getRed =  0b01110000;
const byte caseZero = 0b00000000;
byte MO;
byte MI;

uint8_t Blue[] = { 0, 0, 0, 0 };
uint8_t  Red[] = { 0, 0, 0, 0 };
int sumBlue = 0, prevBlue=0;
int sumRed  = 0, prevRed=0;
unsigned long int loopCount=0, ignoreCount=0;
unsigned long int minIgnore=1023, maxIgnore=0;

void setupSPI() {
  Serial.println("Vitruvian Bots Team 4201 Score Keeper Simulation");
  Serial.println("K.Urashima, jLeBlanc");
  Serial.println("Begin simulation .....");
  pinMode(SCK, OUTPUT);
  pinMode(MOSI, OUTPUT);
  digitalWrite(SCK, LOW);
  digitalWrite(MOSI, LOW);

  // disable every peripheral 
  for(int j = 0; j<=numSS-1; j++) {
    pinMode(SSpin[j], OUTPUT);
    digitalWrite(SSpin[j], HIGH);
  }
  
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV8);   
  
  for(int j = 0; j<=numSS-1; j++) {
    digitalWrite(SSpin[j], LOW);
//    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
    delayMicroseconds(20);
    MO = initSlave0;
    SPI.transfer(MO);  //Set slave to zero counters
    delayMicroseconds(20);
    MO = initSlave1;
    Serial.print("MO after initSlave1="); Serial.println(MO); 
    MI = SPI.transfer(MO);
    Serial.print("Remote ");
    Serial.print( j+1 );
    Serial.print(" on SS pin ");
    Serial.print(SSpin[j]);
    if(MI == initSlave0){      
      Serial.println(" READY ");
    }
    else {
      Serial.print(" MI=");
      Serial.print(MI);
      Serial.println(" FAILED to Ready");
    }
    delayMicroseconds(20);
    digitalWrite(SSpin[j], HIGH);
//    SPI.endTransaction();
   }
   Serial.println("setupSPI complete");
}

//*************************************************** 
//  This is a library for our I2C LED Backpacks
// * Adafruit LED backpack HT16K33 
// * https://learn.adafruit.com/adafruit-led-backpack/0-dot-56-seven-segment-backpack-arduino-setup 
// * A5 Connect CLK to the I2C clock - on Arduino UNO thats Analog #5 
// * A4 Connect DAT to the I2C data - on Arduino UNO thats Analog #4 
//
// Enable one of these two #includes and comment out the other.
// Conditional #include doesn't work due to Arduino IDE shenanigans.
#include <Wire.h> // Enable this line if using Arduino Uno, Mega, etc.
//#include <TinyWireM.h> // Enable this line if using Adafruit Trinket, Gemma, etc.
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"
Adafruit_7segment matrix1 = Adafruit_7segment();
Adafruit_7segment matrix2 = Adafruit_7segment();

void setupLEDmatrix() {
  // LED matrix backpack 
  matrix1.begin(0x70);
  matrix1.println(70);
  matrix1.writeDisplay();
  matrix2.begin(0x71);    
  matrix2.println(71);
  matrix2.writeDisplay();
  Serial.println("setupLEDmatrix complete");
}

// each main scoreboard has 2 control pins; incr and decr 
// when you hold the decr pin high for 1-2 seconds, it resets score=0 
// we use 1 main scoreboard to display red score and 1 for blue score 
// concidentally the incr pin is connected to a red push button (not implying red score!) 
void setupMainScoreboard() {
  for(int i=0; i<6; i++) {
    digitalWrite(MAIN_RED_PIN, HIGH);
    delay(mainScoreFlashTime);
    digitalWrite(MAIN_RED_PIN, LOW);
    delay(mainScoreRestTime);
    Serial.print("CHECK main scoreboard count: "); Serial.println(i);
  }
  digitalWrite(MAIN_RED_PIN, LOW);
  digitalWrite(MAIN_BLUE_PIN, LOW);
  delay(mainScoreRestTime);
  digitalWrite(RESET_RED_PIN, HIGH);
  digitalWrite(RESET_BLUE_PIN, HIGH);
  delay(5000);
  digitalWrite(RESET_RED_PIN, LOW);
  digitalWrite(RESET_BLUE_PIN, LOW);
  delay(100);
  pinMode(RESET_RED_PIN, INPUT);
  pinMode(RESET_BLUE_PIN, INPUT);
 
  Serial.println("setupMainScoreboard complete. ASSERT score=0");
}

void setup() {
  Serial.begin(115200);
  delay(1);
  Serial.println(myPurpose);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(MAIN_RED_PIN, OUTPUT);
  pinMode(MAIN_BLUE_PIN, OUTPUT);
  pinMode(RESET_RED_PIN, OUTPUT);
  pinMode(RESET_BLUE_PIN, OUTPUT);
  setupMainScoreboard();
  setupSPI();
  setupLEDmatrix();
}

byte limitNoise(byte n, int ss) {
  if( n > WILD_LEAP ) {
    ignoreCount++;
    minIgnore = min(n, minIgnore);
    maxIgnore = max(n, maxIgnore);
    Serial.print("IGNORING SS#"); Serial.print(ss);
    Serial.print(" WILD_LEAP=");  Serial.print(WILD_LEAP);
    Serial.print(" minIgnore=");    Serial.print(minIgnore);  
    Serial.print(" maxIgnore=");    Serial.print(maxIgnore);  
    Serial.print(" ignoreCount=");  Serial.print(ignoreCount);  
    Serial.print(". ignoring ");  Serial.println(n);
    return 0; 
  }
  else return n;
}

void showSumScores(int sumRed, int sumBlue, int prevRed, int prevBlue) {
  if( sumRed > prevRed+1 || sumBlue > prevBlue+1 ) {
    Serial.print("loop sumRed: ");
    Serial.print(sumRed);
    Serial.print(" \t");
    Serial.print("sumBlue: ");
    Serial.print(sumBlue);
    Serial.println(".");
  }
  matrix1.println(sumRed);    matrix1.writeDisplay();
  matrix2.println(sumBlue);   matrix2.writeDisplay();
}

int updateMainScore(byte color, int curr, int prev) {
  int pin=0;
  if( color == RED ) {  pin = MAIN_RED_PIN; }
  else { pin = MAIN_BLUE_PIN; }
  // KLUDGE IGNORE "IMPOSSIBLE" WILD LEAPS 
  if( abs(curr - prev) > WILD_LEAP ) {
    Serial.print("WARNING updateMainScore KLUDGE IGNORE \"IMPOSSIBLE\" WILD_LEAP=");
    Serial.println(WILD_LEAP);
    return prev;
  }
  
  while( prev < curr ) {
    // DEBUG INFO 
//    if( color == RED ) {
      Serial.print("INCR pin=");  Serial.print(pin);      
      Serial.print(" prev="); Serial.print(prev);   Serial.print(" curr="); Serial.println(curr);
//    }
    digitalWrite( pin, HIGH );
    delay(mainScoreFlashTime);
    digitalWrite( pin, LOW );
    delay(mainScoreRestTime);
    ++prev;
  }
  return prev;
}

// collect scores from each peripheral [Slave] 
// note heavy dependence on global vars
void collectScoreForEachPeripheral() {
  for(int j = 0; j<=numSS-1; j++){
    digitalWrite(SSpin[j], LOW);
//    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
    delayMicroseconds(20);
    MO = getBlue;
    SPI.transfer(MO);      // Send cmd for Blue data
    delayMicroseconds(20);
    MO = getRed;
    MI = SPI.transfer(MO); // Receive Blue and send cmd for Red data
    Blue[j] = limitNoise(MI, SSpin[j]);
    delayMicroseconds(20);
    MO = caseZero;
    MI = SPI.transfer(MO); // Receive Red data
    Red[j] = limitNoise(MI, SSpin[j]);
    delayMicroseconds(20);
    digitalWrite(SSpin[j], HIGH);
    delayMicroseconds(20); // orig 25);
//    SPI.endTransaction();
    sumBlue +=Blue[j];
    sumRed  +=Red[j];
  }
}

void loop() {
  loopCount++;
  prevBlue = sumBlue;
  prevRed = sumRed;
  sumBlue = 0;
  sumRed = 0;
  collectScoreForEachPeripheral();
  showSumScores(sumRed, sumBlue, prevRed, prevBlue);
  sumRed = updateMainScore(RED, sumRed, prevRed); 
  sumBlue = updateMainScore(BLUE, sumBlue, prevBlue); 
  delay(500);  //Simulates time between scoring events
}

void TESTloop() {
  digitalWrite(MAIN_RED_PIN, HIGH);
  delay(100);
  digitalWrite(MAIN_RED_PIN, LOW);
  delay(300);
  Serial.println(loopCount++);
}
