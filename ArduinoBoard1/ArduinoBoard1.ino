#include "Definitions.h"
//Source Code Version

#define FIRMWARE_MAJOR_VERSION 0
#define FIRMWARE_MINOR_VERSION 0
#define FIRMWARE_BUILD_NUMBER 0

//Configuration Defines
#define BOARD_ID 18
#define BOARD_TYPE BOARDTYPE_ARDUINOMEGA
#define PRINT_DEBUG_LINES 1
#define SHIELD1_TYPE SHIELDTYPE_NONE
#define SHIELD2_TYPE SHIELDTYPE_NONE
#define SHIELD3_TYPE SHIELDTYPE_NONE
#define SHIELD4_TYPE SHIELDTYPE_NONE

//External Libraries
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

//Defines
#define LEDS_PER_STRIP 8
#define LED_STRIPS 4

//Pin Definitions
#define LEDPIN 0

//Create Objects
Adafruit_NeoPixel led_strips = Adafruit_NeoPixel(LEDS_PER_STRIP*LED_STRIPS, LEDPIN, NEO_RGBW + NEO_KHZ800); //FOR PN: 600004

//Function Definitions
bool init_ledstrips();
void setup() {
  init_ledstrips();

}

void loop() {
  // put your main code here, to run repeatedly:

}
bool init_ledstrips()
{
  led_strips.begin();
  led_strips.show(); // Initialize all pixels to 'off'
  for(uint16_t i=0; i<led_strips.numPixels(); i++) 
  {
    led_strips.setPixelColor(i, led_strips.Color(255, 0, 0));
    led_strips.show();
    delay(1);
  }
}
