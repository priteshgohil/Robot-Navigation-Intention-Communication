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
#define DELAY_1S 1000
#define DELAY_500MS 500
#define DELAY_300MS delay(300)
#define DELAY_200MS delay(200)

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
char data_1[32]={0};
char data_2[32]={0};
char cRxMsg[RX_MSG_SIZE];
//char delim[] = "#;:";
int data_3=0;

void setup() {
  // put your setup code here, to run once:
 CircuitPlayground.begin();
 Serial.begin(BAUD_RATE);
 Serial.println("<Arduino is ready>");
}

void loop() {
  // put your main code here, to run repeatedly:
 Is_data_available();
 Parse_Message();
 Execute_Action();
}

void Is_data_available() {
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

void Parse_Message(){
  if(UARTmsg.bnewData){
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
  //        strcpy(packet1,ptr);
          ptr = strtok(NULL, delim); //get LED numbers
          UARTmsg.unLed_Number = strtol(ptr, NULL, 16);
          Serial.println(ptr);
          ptr = strtok(NULL, delim); //get LED color
          UARTmsg.unLed_Colour = strtol(ptr, NULL, 16);   //converting hex string to int
          Serial.println(ptr);
        }
        else if(strcmp(ptr,"PAT")==0){
  //        strcpy(packet2,ptr);
          ptr = strtok(NULL, delim); //get LED blinking pattern
          UARTmsg.unBlink_Pattern = strtol(ptr, NULL, 16);
        }
        else if(strcmp(ptr,"BUZ")==0){
  //        strcpy(packet3,ptr);
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
}


void Execute_Action(){
  if(UARTmsg.unBlink_Pattern == 0x1000){
    movingRight(UARTmsg.unLed_Number,UARTmsg.unLed_Colour);
  }
  else if(UARTmsg.unBlink_Pattern == 0x1001){
    blickGeneral(UARTmsg.unLed_Number,UARTmsg.unLed_Colour);
  }
  else if(UARTmsg.unBlink_Pattern == 0x1002){
    blickGeneral(UARTmsg.unLed_Number,UARTmsg.unLed_Colour);
  }
}


void blickGeneral(uint16_t LedNumber, uint32_t colour){
  CircuitPlayground.clearPixels();              //clear all the LED pixels
  delay(300);
  bool singleBit = 0;
  for (int i = 0; i<10; i++){
    singleBit = (LedNumber >> i) & 1;
    if (singleBit){
      CircuitPlayground.setPixelColor(i,colour);
    }
  }
  DELAY_300MS;
}

void forwardMove() {
  CircuitPlayground.setPixelColor(2,0,GREEN,0);  //green
  CircuitPlayground.setPixelColor(7,0,GREEN,0);  //green
  DELAY_300MS;
  CircuitPlayground.setPixelColor(3,0,GREEN,0);  //green
  CircuitPlayground.setPixelColor(6,0,GREEN,0);  //green
  DELAY_300MS;
  CircuitPlayground.setPixelColor(4,0,GREEN,0);  //green
  CircuitPlayground.setPixelColor(5,0,GREEN,0);  //green
//  speakerTone();
  DELAY_300MS;
}

void movingRight(uint16_t LedNumber, uint32_t colour){
  bool pixel1 = 0, pixel2 = 0;
  uint8_t cutIndex = 5;
  const uint16_t rightMask = 0x001F;
  const uint16_t leftMask = 0x03E0;
  uint16_t led5to9 = (LedNumber & leftMask)>>cutIndex; 
  uint16_t led0to4 = (LedNumber & rightMask)<<3; 

  CircuitPlayground.clearPixels();              //clear all the LED pixels
  delay(150);
  for (int i = 0, j=cutIndex-1; i<cutIndex; i++,j--){
    pixel1 = (led0to4 << i) & 0x80;       //checking LED number form position 4 onwards , 3,2,1,0
    pixel2 = (led5to9 >> i) & 1;          //checking LED number from poition 5 onwards ,6,7,8,9

    if(pixel1){
      CircuitPlayground.setPixelColor(i+cutIndex,colour);     //i+curIndex, because actual index is 0+5 (remember that data is shifted to LSB)
    }
    if(pixel2){
      CircuitPlayground.setPixelColor(j,colour);
    }
    delay(300);
  }
}
