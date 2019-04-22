#include <Adafruit_CircuitPlayground.h>
#include "LED_config.h"

#define DELAY_1S 1000
#define DELAY_500MS 500
#define DELAY_300MS delay(300)
#define DELAY_200MS delay(200)

enum ForwardPattern {
  STAGE1 = 0b0010000100,
  STAGE2 = 0b0001001000,
  STAGE3 = 0b0000110000
};

int leds[10] =  {0,1,2,3,4,5,6,7,8,9};
//LSB is led0 and MSB is led 9
int forward = 0b11110000;

void setup() {
  // put your setup code here, to run once:
  CircuitPlayground.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  CircuitPlayground.clearPixels();              //clear all the LED pixels
  delay(500);
  //setPixelColor(LED#,R,G,B)
//  CircuitPlayground.setPixelColor(0,255,0,0);  //red
//  CircuitPlayground.setPixelColor(1,0,255,0);  //green
//  CircuitPlayground.setPixelColor(2,0,0,255);  //blue
//  CircuitPlayground.setPixelColor(3,255,0,255);  //purple
//  CircuitPlayground.setPixelColor(4,255,255,0);  //yellow
//  CircuitPlayground.setPixelColor(5,0,255,255);  //cyan
//  CircuitPlayground.setPixelColor(9,RED,0,0);  //red
    forwardMove();
    CircuitPlayground.strip.setPixelColor

}

void forwardMove() {
  CircuitPlayground.setPixelColor(2,0,GREEN,0);  //green
  CircuitPlayground.setPixelColor(7,0,GREEN,0);  //green
  DELAY_300MS;
  CircuitPlayground.setPixelColor(3,RED,0,0);  //green
  CircuitPlayground.setPixelColor(6,RED,0,0);  //green
  DELAY_300MS;
  CircuitPlayground.setPixelColor(4,0,0,BLUE);  //green
  CircuitPlayground.setPixelColor(5,0,0,BLUE);  //green
//  speakerTone();
  DELAY_300MS;
}

void speakerTone(){
  CircuitPlayground.playTone(440,500);
}
