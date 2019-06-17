/*
  RobotIntentionCommunication

  This program receives the serial input from the robot with predefined message frame.
  Module:
  1. Read the data from serial port and store it as a string: isDataAvailable();
  2. Parse the received message and store it in the structure: parseMessage();
  3. Check for the pattern received, and perform relavent action from data: executeAction();

  Created: 15 April 2019
  By: Pritesh Gohil and Eduardo Cervantes
  Modified: 05 May 2019
  By: Pritesh Gohil and Eduardo Cervantes

  References: 
  https://www.codingame.com/playgrounds/14213/how-to-play-with-strings-in-c/string-split

*/

#include <Adafruit_CircuitPlayground.h>
//#include "Message.h"
#define BAUD_RATE 9600
#define RX_MSG_SIZE 100
#define RED 255 
#define BLUE 255 
#define GREEN 255 

typedef struct{
    uint16_t unLed_Number;
    uint32_t unLed_Colour;
    uint16_t unBlink_Pattern;
    uint16_t unBuzz_Freq;
    uint8_t unBuzz_time;
    uint16_t unBuzz_Pattern;
    bool bnewData;
    bool bDataAvailable;
    uint8_t unRxSize;
}stMessage;

stMessage UARTmsg;
const byte numChars = 100;
char receivedChars[numChars]; // an array to store the received data
bool execute = false;
char cRxMsg[RX_MSG_SIZE];

void setup() {
  // put your setup code here, to run once:
 CircuitPlayground.begin();
 //Initialize the UART port
 Serial.begin(BAUD_RATE);
 Serial.println("<Arduino is ready>");
}

void loop() {
  // put your main code here, to run repeatedly:
  //chekc if data from UART is available
 isDataAvailable();
 //if data is available then parse the received message
 if(UARTmsg.bnewData){
  parseMessage();
 }
 executeAction();
// executeCmdVel();
}

void isDataAvailable() {
 static uint8_t index = 0;
 char endline = '>';
 char rxChar;
 
  if (Serial.available()) {
    while (Serial.available() > 0 && UARTmsg.bnewData == false) {
      rxChar = Serial.read(); //Also Serial.readString(); is available but parsing the data should be implemented in a different way
      Serial.print(rxChar);
      if (rxChar != endline) 
      {
         cRxMsg[index] = rxChar;
         index++;
          if (index >= numChars) 
            {index = numChars - 1;}
      }
      else 
      {
         Serial.print("end of string");
         cRxMsg[index] = '\0'; // terminate the string
         index = 0;
         UARTmsg.bnewData = true;
         }
       }
     rxChar = Serial.read();
     }
 }

void parseMessage(){
    Serial.println("parse");
    char delim[] = "#;:";
    char *ptr = strtok(cRxMsg, delim);
    while(ptr != NULL)
    {
      if (strcmp(ptr, "<RIC")==0){
        Serial.println("Removed; <RIC\n");
      }
      else if(strcmp(ptr, "RIC>")==0 ){
        Serial.println("Removed: RIC>\n");
      }
      else{
  //      strcpy(msg, ptr);
        if(strcmp(ptr,"LED")==0){
          ptr = strtok(NULL, delim); //get LED numbers
          UARTmsg.unLed_Number = strtol(ptr, NULL, 16);
          Serial.println(ptr);
          ptr = strtok(NULL, delim); //get LED color
          UARTmsg.unLed_Colour = strtol(ptr, NULL, 16);   //converting hex string to int
          Serial.println(ptr);
        }
        else if(strcmp(ptr,"PAT")==0){
          ptr = strtok(NULL, delim); //get LED blinking pattern
          UARTmsg.unBlink_Pattern = strtol(ptr, NULL, 16);
        }
        else if(strcmp(ptr,"BUZ")==0){
          ptr = strtok(NULL, delim); //get Buzzer frequency
          UARTmsg.unBuzz_Freq = strtol(ptr, NULL, 16);
          ptr = strtok(NULL, delim); //get Buzz time
          UARTmsg.unBuzz_time = strtol(ptr, NULL, 16);
          ptr = strtok(NULL, delim); //get Buzz style
          UARTmsg.unBuzz_Pattern = strtol(ptr, NULL, 16);
        }
      }
      ptr = strtok(NULL, delim);
      }
      UARTmsg.bnewData = false;
      UARTmsg.bDataAvailable = true;
}


void executeCmdVel(){
  blickGeneral(UARTmsg.unLed_Number, UARTmsg.unLed_Colour);
}

void executeAction(){
  if(UARTmsg.unBlink_Pattern == 0x1000){
    movingStraight(UARTmsg.unLed_Number,UARTmsg.unLed_Colour);
  }
  else if(UARTmsg.unBlink_Pattern == 0x1001){
    movingReverse(UARTmsg.unLed_Number,UARTmsg.unLed_Colour);
  }
  else if(UARTmsg.unBlink_Pattern == 0x1002){
    blickGeneral(UARTmsg.unLed_Number,UARTmsg.unLed_Colour);
  }
  else if(UARTmsg.unBlink_Pattern == 0x1003){
    blickGeneral(UARTmsg.unLed_Number,UARTmsg.unLed_Colour);
    Buzzer(UARTmsg.unBuzz_time,UARTmsg.unBuzz_Freq);
  }
  else if(UARTmsg.unBlink_Pattern == 0x1004){
    movingDiagonal(UARTmsg.unLed_Number,UARTmsg.unLed_Colour);
    Buzzer(UARTmsg.unBuzz_time,UARTmsg.unBuzz_Freq);
  }
  else if(UARTmsg.unBlink_Pattern == 0x1005){
    movingDiagonal(UARTmsg.unLed_Number,UARTmsg.unLed_Colour);
    Buzzer(UARTmsg.unBuzz_time,UARTmsg.unBuzz_Freq);
  }
  else if(UARTmsg.unBlink_Pattern == 0x1006){
    movingDiagonal(UARTmsg.unLed_Number,UARTmsg.unLed_Colour);
    Buzzer(UARTmsg.unBuzz_time,UARTmsg.unBuzz_Freq);
  }
  else if(UARTmsg.unBlink_Pattern == 0x1007){
    movingDiagonal(UARTmsg.unLed_Number,UARTmsg.unLed_Colour);
    Buzzer(UARTmsg.unBuzz_time,UARTmsg.unBuzz_Freq);
  }
  else if(UARTmsg.unBlink_Pattern == 0x1008){
    rotateCW(UARTmsg.unLed_Number,UARTmsg.unLed_Colour);
  }
  else if(UARTmsg.unBlink_Pattern == 0x1009){
    rotateACW(UARTmsg.unLed_Number,UARTmsg.unLed_Colour);
  }
  else if(UARTmsg.unBlink_Pattern == 0x1010){
    docking(UARTmsg.unLed_Number,UARTmsg.unLed_Colour);
  }
  else if(UARTmsg.unBlink_Pattern == 0x1020){
    blickGeneral(UARTmsg.unLed_Number,UARTmsg.unLed_Colour);
  }
}


void blickGeneral(uint16_t LedNumber, uint32_t colour){
  CircuitPlayground.clearPixels();              //clear all the LED pixels
  delay(150);
  bool singleBit = 0;
  if(LedNumber){
    for (int i = 0; i<10; i++){
    singleBit = (LedNumber >> i) & 1;
    if (singleBit){
      CircuitPlayground.setPixelColor(i,colour);
    }
  }
  delay(200); 
  }
}


//LedNumber = 0,0,0,0,0,0,L9,L8,L7,L6,L5,L4,L3,L2,L1,L0 = 0x03FF h

void movingStraight(uint16_t LedNumber, uint32_t colour){
  bool pixel1 = 0, pixel2 = 0;
  uint8_t cutIndex = 5;                   //dividing LED's into group of 2. 1st: 0 to 4 and 2md: 5 to 9
  const uint16_t rightMask = 0x001F;      //Right half of the LEDs are 0 to 4. and its first 5 LSB of LED number so mask is 0x001F
  const uint16_t leftMask = 0x03E0;       //Left half of the LEDs are 5 to 9. and its next 5 LSB of LED number so mask is 0x03E0
  uint16_t led5to9 = (LedNumber & leftMask)>>cutIndex;    //bcz our LED pattern is mid highest to lowest(i.e. first blink LED 2-7 -> 1-6 ->0-5), 
                                                          //we are moving LED 5 to 9 to LSB position, and perform shift right operation to get highest number of LED first
  uint16_t led0to4 = (LedNumber & rightMask)<<3;          //we are moving LED 0 to 4 to MSB position, and perform shift left operation to get highest number of LED first

  CircuitPlayground.clearPixels();              //clear all the LED pixels
  delay(150);
  for (int i = 0, j=cutIndex-1; i<cutIndex; i++,j--){
    pixel1 = (led0to4 << i) & 0x80;       //converting our data to uint8_t, checking LED number form position 4 onwards , 3,2,1,0. Check if it is true or not by 0x10000000 (because LED at MSB position)
    pixel2 = (led5to9 >> i) & 1;          //checking LED number from poition 5 onwards ,6,7,8,9. Check if it is true or not by 0x00000001 (because LED at LSB position)

    if(pixel1){
      CircuitPlayground.setPixelColor(i+cutIndex,colour);     //i+curIndex, because actual index is 0+5 (remember that data is shifted to LSB)
    }
    if(pixel2){
      CircuitPlayground.setPixelColor(j,colour);
    }
    delay(300);
  }
}

void movingReverse(uint16_t LedNumber, uint32_t colour){
  bool pixel1 = 0, pixel2 = 0;
  uint8_t cutIndex = 5;
  const uint16_t rightMask = 0x001F;
  const uint16_t leftMask = 0x03E0;
  uint16_t led5to9 = (LedNumber & leftMask)>>2; 
  uint16_t led0to4 = (LedNumber & rightMask); 

  CircuitPlayground.clearPixels();           //clear all the LED pixels
  delay(150);
  for (int i = 0, j=cutIndex-1; i<cutIndex; i++,j--){
    pixel1 = (led0to4 >> i) & 1;             //checking LED number form position 4 onwards , 3,2,1,0
    pixel2 = (led5to9 << i) & 0x80;          //checking LED number from poition 5 onwards ,6,7,8,9

    if(pixel1){
      CircuitPlayground.setPixelColor(i,colour);     //i+curIndex, because actual index is 0+5 (remember that data is shifted to LSB)
    }
    if(pixel2){
      CircuitPlayground.setPixelColor(j+cutIndex,colour);
    }
    delay(300);
  }
}

void movingDiagonal(uint16_t LedNumber, uint32_t colour){
  CircuitPlayground.clearPixels();              //clear all the LED pixels
  delay(150);
  bool singleBit = 0;
  for (int i = 0; i<10; i++){
    singleBit = (LedNumber >> i) & 1;
    if (singleBit){
      CircuitPlayground.setPixelColor(i,colour);
    }
  }
  delay(200);
  }

void rotateACW(uint16_t LedNumber, uint32_t colour) {
  // Can be any two pixels
  int pixel1 = 0;
  int pixel2 = 5;
  while(pixel2 <10){
    CircuitPlayground.clearPixels();
    // Turn on two pixels to SPIN_COLOR
    CircuitPlayground.setPixelColor(pixel1, colour);
    CircuitPlayground.setPixelColor(pixel2, colour);


    CircuitPlayground.setPixelColor(pixel1+1, colour);
    CircuitPlayground.setPixelColor(pixel2+1, colour);
    // Increment pixels to move them around the board
    pixel1 = pixel1 + 1;
    pixel2 = pixel2 + 1;
    
    // Wait a little bit so we don't spin too fast
    delay(100);
  }
}

void rotateCW(uint16_t LedNumber, uint32_t colour) {
  // Can be any two pixels
  int pixel1 = 4;
  int pixel2 = 9;
  while(pixel2 >4){
    CircuitPlayground.clearPixels();
    // Turn on two pixels to SPIN_COLOR
    CircuitPlayground.setPixelColor(pixel1, colour);
    CircuitPlayground.setPixelColor(pixel2, colour);
  
    // Increment pixels to move them around the board
    pixel1 = pixel1 - 1;
    pixel2 = pixel2 - 1;
  
    // Wait a little bit so we don't spin too fast
    delay(100);
  }
}

void docking(uint16_t LedNumber, uint32_t colour) {
  int pixel1 = 0;
  int pixel2 = 9;
  
  while (1) {
    // Scan in one direction
    for (int step=0; step<4; step++) {
      CircuitPlayground.clearPixels();
    
      CircuitPlayground.setPixelColor(pixel1, colour);
      CircuitPlayground.setPixelColor(pixel2, colour);
  
      pixel1 = pixel1 + 1;
      pixel2 = pixel2 - 1;
      
      delay(100);    
    }
  
    // Scan back the other direction
    for (int step=0; step<4; step++) {
      CircuitPlayground.clearPixels();
    
      CircuitPlayground.setPixelColor(pixel1, colour);
      CircuitPlayground.setPixelColor(pixel2, colour);
  
      pixel1 = pixel1 - 1;
      pixel2 = pixel2 + 1;
      
      delay(100);
    }
    break;
  }
}


void Buzzer(uint16_t Buzz_Freq, uint8_t Buzz_time)
{
  CircuitPlayground.playTone(Buzz_time,Buzz_Freq);
  }
