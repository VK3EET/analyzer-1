/* 
Sketch for standalone analyzer using Arduino Uno and AA-30.Zero
with PC passthrough mode

Collated from other sources by Mark Valiukas, VK3EET

Standalone analyzer code derived from work by Alexander Antonov published at:
https://rigexpert.com/arduino-aa-30-zero-tiny-analyzer/
© Alexander Antonov UR4MCB February 05, 2020.

Encoder code derived from work published at:
https://www.instructables.com/id/Improved-Arduino-Rotary-Encoder-Reading/

Pressing and holding the encoder while starting launches the
PC passthrough mode.

The analyzer operates in the frequency range from 100kHz to 230 MHz.
The operating frequency is set by rotating the encoder. 
Measurement is started by pressing and then releasing the encoder knob once.
One click of the encoder turn changes the frequency by 10 kHz,
One click of the turn with push of encoder knob changes the frequency by 1 MHz.
Information is displayed on the LCD type 2004 I2C.

Used libraries:
  For display operation: https://rigexpert.com/files/libraries/LiquidCrystal_I2C_V112/
  For the analyzer to work: https://rigexpert.com/files/libraries/RigExpertZero/

The original UR4MCB code didn't work properly with my encoder - 20kHz/2MHz 
increments on a full detent turn, with a half detent turn resulting on 10kHz/1MHz 
increments. So, I dropped the original encoder code and incorporated code derived 
from the SimonM83 example found at Instructables. This encoder code looks for a
rising edge to indicate that the encoder has arrived at a detent, rather than simply
looking for changes.

Tested with:
Arduino Uno clone, unbranded.
RigExpert AA-30.ZERO
2004 I2C LCD
Rotary encoder, Jaycar catalogue number XC3736 


To do:
  um, lots? This was just a quick-and-dirty hack to get something functional
  with the hardware I had on hand.

Version History:
0.01  20200308

*/


/* Added a library to work with the display. */
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/* Display setting. Address I2C, number of columns, number of rows. */
//adjust I2C address and parameters to suit your device - this is for a 2004 at 0x3F
LiquidCrystal_I2C lcd(0x3F, 20, 4);

/* Added a library to work with AA-30 ZERO analyzer */
#include "RigExpertZero.h"

/* ZERO pins */
#define RX_PIN 4
#define TX_PIN 7

RigExpertZero ZERO(RX_PIN, TX_PIN);

/* encoder pins */
#define SW 5
static int pinA = 2; // Our first hardware interrupt pin is digital pin 2
static int pinB = 3; // Our second hardware interrupt pin is digital pin 3

/* variables used for encoder data processing */
volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile byte encoderDir = 0; //this variable stores our current value of direction, encoder position.
volatile byte oldEncDir = 0; //stores the last encoder direction value so we can compare to the current reading and see if it has changed
volatile byte encoderMove = 0; //has encoder moved? 
volatile byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent
boolean swState; //switch state for encoder push button
boolean prevswState = true; // previous encoder push button switch state

/* setting the center frequency of the measurement, Hz */
volatile int32_t freq = 15000000;

/* variable "timer" so as not to use the standard delay function */
int32_t timer = millis();

/* separate softwareserial instance for passthrough mode */
//SoftwareSerial ZEROPT(RX_PIN, TX_PIN);  // RX, TX for passthrough

void setup() {

  //encoder setup 
  pinMode(pinA, INPUT_PULLUP); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinB, INPUT_PULLUP); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  attachInterrupt(0,PinA,RISING); // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  attachInterrupt(1,PinB,RISING); // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)


  ZERO.startZero();                   // Initialize analyzer
  delay(50);                          // required delay
  lcd.init();                         // Initialize display
  delay(10);                          // required delay
  lcd.backlight();                    // Turn on display backlight

/* Check if the analyzer is found? */
  while (!ZERO.startZero()) {
    lcd.setCursor(6, 1);              // set the cursor
    lcd.print("analyzer");            // and print text on the display
    lcd.setCursor(6, 2);
    lcd.print("not found");
    delay(1000);                      // recheck after 1 second
  }

  


/* If found, then display the greeting on the screen. */
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("AA-30.ZERO analyzer");
  //lcd.setCursor(0, 1);
  //lcd.print("  antenna analyzer  ");
  lcd.setCursor(0, 2);
  lcd.print(" (press+hold encoder");
  lcd.setCursor(0, 3);
  lcd.print(" switch for PC mode)");
  //lcd.print("--------------------");
  

  delay(4000);                         // Display splash screen 4 seconds
  swState = bitRead(PIND, SW);         // Read encoder button state for PC mode selection
  lcd.clear();                         // Clear display

  //encoder button press pulls line down
  if (swState == false) {              //switch to PC mode
    /*  
     *   kill interrupts - they are not needed for passthrough, and 
     *   in testing it was found that they could interfere with data 
     *   transfer
     */
    detachInterrupt(digitalPinToInterrupt(pinA));
    detachInterrupt(digitalPinToInterrupt(pinB));

    /*    
     *    Need a separate SoftwareSerial instance for passthrough,
     *    outside the analyzer library code. ZERO_PT (for PassThrough)
     */
    SoftwareSerial ZERO_PT(RX_PIN, TX_PIN);  // RX, TX
    ZERO_PT.begin(38400);     // init AA side UART
    ZERO_PT.flush();
    Serial.begin(38400);      // init PC side UART
    Serial.flush(); 
    lcd.setCursor(0, 1);
    lcd.print("PC mode"); 
    lcd.setCursor(0,2);
    lcd.print("- power off to exit");
    /* 
     *  Nice low-latency loop - never gets to main loop if PC mode activated. 
     */
    while (swState == false) { 
      if (ZERO_PT.available()) Serial.write(ZERO_PT.read());  // data stream from AA to PC
      if (Serial.available()) ZERO_PT.write(Serial.read());  // data stream from PC to AA
    }
  } 
  else {                              //standalone mode
    lcd.setCursor(0, 1);
    lcd.print("press to measure");
  }
}

void PinA(){
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
  if(reading == B00001100 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderDir = 1; //decrement the encoder's position count
    encoderMove = 1;
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00000100) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

void PinB(){
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; //read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderDir = 2; //increment the encoder's position count
    encoderMove = 1;
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00001000) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
  sei(); //restart interrupts
}


void loop() {

  processEncoder();
  updateScreen();
   
}

void processEncoder() {
  /*  encoder push switch pulls down line, so 
   *  true when switch is open 
   *  false when switch is closed 
   */
  swState = bitRead(PIND, SW);

  /* Rotary encoder processing */
  // left, no button
  if ( (encoderMove == 1) && (encoderDir == 2) && (swState == true) ) {
    freq_left();
    encoderMove = 0;
  }

  // right, no button
  if ( (encoderMove == 1) && (encoderDir == 1) && (swState == true) ) {
    freq_right();
    encoderMove = 0;
  }
  
  // left, holding button
  if ( (encoderMove == 1) && (encoderDir == 2) && (swState == false) ) {
    freq_leftH();
    encoderMove = 0;
    prevswState = false;
  }
  
  //right, holding button
  if ( (encoderMove == 1) && (encoderDir == 1) && (swState == false) ) {
    freq_rightH();
    encoderMove = 0;
    prevswState = false;
  }
  
  if ( swState == false ) {
    prevswState = false;
  }
  
  //if (enc.isClick()) {
  if ( (swState == true) && (prevswState == false ) ) {
    start_m ();
    prevswState = true;
  }  
}


void updateScreen () {
/* Update the frequency value on screen every 200 ms. */
/*
  RigExpert have removed the frequency limits in recent AA-30.ZERO firmware.
  Nominal specs are 60kHz to 35MHz, but preliminary testing into a 50 ohm
  calibration load obtained with a NanoVNA shows a 1:1 SWR up to 500MHz.

  Of course, *way* outside the nominal range, results are not likely to be 
  terribly reliable or consistent for real loads that are far from 50 ohms.
  My testing into known 2m antennas showed comparable results to other
  instruments, but the results at 70cm weren't believable. 
  
  This display code should work from 10kHz through 999.999MHz to facilitate
  experimentation, testing, and re-use. UR4MCB stated in a post in a qrz.com
  forum that the device should be good to 230MHz, but that things may get a
  little hairy in a few places outside the amateur bands with spurs at 112 and
  150 MHz - https://forums.qrz.com/index.php?threads/a-little-more-about-the-aa-30-zero-from-rigexpert.691567/ 
*/
  if (millis() - timer > 200) {
    if ((freq / 1000) > 100000) {   //values above 100MHz
      lcd.setCursor(0, 0);
      lcd.print("FQ = ");
      lcd.setCursor(4, 0);
      lcd.print(freq / 1000);
      lcd.setCursor(11, 0);
      lcd.print("kHz");
      timer = millis();
    }    
    if ( ((freq / 1000) < 99999) && ((freq / 1000) >= 10000) ) {     //values between 10MHz and 99.999MHz
      lcd.setCursor(0, 0);
      lcd.print("FQ = ");
      lcd.setCursor(4, 0);
      lcd.print(' ');
      lcd.setCursor(5, 0);
      lcd.print(freq / 1000);
      lcd.setCursor(11, 0);
      lcd.print("kHz");
      timer = millis();
    }
    if ( ((freq / 1000) < 9999) && ((freq / 1000) >= 1000) ) {  //values between 1MHz and 9.999MHz
      lcd.setCursor(0, 0);
      lcd.print("FQ = ");
      lcd.setCursor(5, 0);
      lcd.print(' ');
      lcd.setCursor(6, 0);
      lcd.print(freq / 1000);
      lcd.setCursor(11, 0);
      lcd.print("kHz");
      timer = millis();
    }
    if ( ((freq / 1000) < 999) && ((freq / 1000) >= 100) ) {   //values below 1MHz but over 100kHz
      lcd.setCursor(0, 0);
      lcd.print("FQ = ");
      lcd.setCursor(6, 0);
      lcd.print(' ');
      lcd.setCursor(7, 0);
      lcd.print(freq / 1000);
      lcd.setCursor(11, 0);
      lcd.print("kHz");
      timer = millis(); 
    }
if ( (freq / 1000) < 100 ) {   //values below 1MHz
      lcd.setCursor(0, 0);
      lcd.print("FQ = ");
      lcd.setCursor(6, 0);
      lcd.print("  ");
      lcd.setCursor(8, 0);
      lcd.print(freq / 1000);
      lcd.setCursor(11, 0);
      lcd.print("kHz");
      timer = millis(); 
    }
    
  }
}

void freq_leftH () {
  freq = freq + 1000000;
  if (freq >= 230000000) {
    freq = 230000000;
  }
}

void freq_rightH () {
  freq = freq - 1000000;
  if (freq <= 100000) {
    freq = 100000;
  }
}

void freq_left () {
  freq = freq + 10000;
  if (freq >= 230000000) {
    freq = 230000000;
  }
}

void freq_right () {
  freq = freq - 10000;
  if (freq <= 100000) {
    freq = 100000;
  }
}

void start_m () {
  ZERO.startMeasure(freq);              // start measurement
  delay(10);
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("SWR");
  float SWR = ZERO.getSWR();            // get SWR value
  lcd.setCursor(4, 1);
  if (SWR < 10) {
    lcd.print(SWR, 2);
  }
  if ((SWR >= 10) && (SWR < 100)) {
    lcd.print(SWR, 1);
  }
  if ((SWR >= 100) && (SWR <= 200)) {
    lcd.print(">100");
  }

  int Z = ZERO.getZ();                  // get Z value
  lcd.setCursor(11, 1);
  lcd.print('Z');
  lcd.setCursor(13, 1);
  if (Z <= 1000) {
    lcd.print(Z);
  }
  if (Z > 1000 ) {
    lcd.print(">1000");
  }

  float RAWR = ZERO.getR();
  lcd.setCursor(0,2);
  lcd.print("R");
  lcd.setCursor(4,2);
  if (RAWR < 10){
    lcd.print(RAWR, 2);
  }
  if ((RAWR >= 10 ) && (RAWR < 100)){
    lcd.print(RAWR, 1);
  }
  if (RAWR >= 100) {
    lcd.print(RAWR,0);
  }

  float RAWX = ZERO.getX();
  lcd.setCursor(11,2);
  lcd.print("X");
  lcd.setCursor(13,2);
  if ((RAWX > -10) && (RAWX < 10)){
    lcd.print(RAWX, 2);
  }
  if ( ((RAWX <= -10) && (RAWX > -100)) || ((RAWX >= 10) && (RAWX < 100)) ){
    lcd.print(RAWX, 1);
  }
  if ( (RAWX <= -100) || (RAWX >= 100)){
    lcd.print(RAWX, 0);
  }

  float RL = ZERO.getRL();
  lcd.setCursor(0,3);
  lcd.print("RL");
  lcd.setCursor(4,3);
  lcd.print(RL);
  //lcd.print(" dB");

}
