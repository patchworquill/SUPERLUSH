#include <Arduino.h>
#include "Buttons.h"
#include <FastLED.h>


//------Notes-------------
#define NOTE_ONE 0
#define NOTE_TWO 12
#define NOTE_THREE 24
#define NOTE_FOUR 36
#define NOTE_FIVE 48
#define NOTE_SIX 60
#define NOTE_SEVEN 72
#define NOTE_EIGHT 84
#define NOTE_NINE 96
#define NOTE_TEN 108

#define NOTE_ON_CMD 0x90
#define NOTE_OFF_CMD 0x80
#define CC_CMD 0xB0
#define MAX_MIDI_VELOCITY 127

//------------------------

#define NUM_KNOBS 6
int analogInputs[NUM_KNOBS];
int analogInputsPrev[NUM_KNOBS];

#define NUM_BUTTONS 10
bool buttons[NUM_BUTTONS];
bool buttonsPrev[NUM_BUTTONS];


Button Button1(2, NOTE_ONE);
Button Button2(3, NOTE_TWO);
Button Button3(4, NOTE_THREE);
Button Button4(5, NOTE_FOUR);
Button Button5(8, NOTE_FIVE);
Button Button6(9, NOTE_SIX);
Button Button7(10, NOTE_SEVEN);
Button Button8(11, NOTE_EIGHT);
Button Button9(12, NOTE_NINE);
Button Button10(13, NOTE_TEN);

Button *Buttons[] {&Button1,&Button2,&Button3,&Button4,&Button5,&Button6,&Button7,&Button8,&Button9,&Button10};

Knob Knob1(0, 20);
Knob Knob2(1, 21);
Knob Knob3(2, 22);
Knob Knob4(3, 23);
Knob Knob5(4, 24);
Knob Knob6(5, 25);

Knob *Knobs[] {&Knob1,&Knob2,&Knob3,&Knob4,&Knob5,&Knob6};

// First led strip, 200 leds, 50 leds in 4 rows 
#define NUM_LEDS_TOP 100 // 98
// Second strip on bottom, 98 leds, 49 in 2 rows 
#define NUM_LEDS_BOTTOM 80

CRGB ledsTop[NUM_LEDS_TOP];
CRGB ledsBottom[NUM_LEDS_BOTTOM];

/************** PINOUT ******************
 *  ----- INPUTS -------
// A0-A5 | POT
// A4 - Center

// 2-5 MICRO SWITCHES - Joystick
// 8-13 MICRO SWITCHES
    // On until button is let go

// INPUTS
// 7 | LEDS_BOTTOM
// 6 | LEDS_TOP

******************************************/

// #define J_UP 3
// #define J_LEFT 2
// #define J_DOWN 12
// #define J_RIGHT 4

// #define PB_1 5 // might be fucked
// #define PB_2 8 // 
// #define PB_3 9 // 
// #define PB_4 10 // might be fucked
// #define PB_5 11 
// #define PB_6 13 

#define LED_PIN_BOTTOM 7
#define LED_PIN_TOP 6

#define MAX_BRIGHTNESS     255
#define FRAMES_PER_SECOND  120

typedef void (*SimplePatternList[])();

uint8_t gHue = 0;                  // rotating "base color" used by many of the patterns
uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current

void confetti()
{
    // random colored speckles that blink in and fade smoothly 
    fadeToBlackBy( ledsTop, NUM_LEDS_TOP, 30); 
    fadeToBlackBy( ledsBottom, NUM_LEDS_BOTTOM, 20);
    int pos = random16(NUM_LEDS_TOP);
    ledsTop[pos] += CHSV(gHue + random8(64), 200, 150);
    // ledsBottom[pos] += CHSV( gHue + random8(64), 200, 150); 
}

// Basically a screensaver
void sinelon()
{
    // a colored dot sweeping back and forth, with fading trails 
    fadeToBlackBy( ledsTop, NUM_LEDS_TOP, 20);
    int pos = beatsin16(13, 0, NUM_LEDS_TOP - 1);
    ledsTop[pos] += CHSV(gHue, 255, 192);
}

void flash_LEDs(){
    for(int i = 0; i < 44; i++){
        ledsBottom[i + 3] += CHSV( gHue + random8(64), 230, 200);
        ledsBottom[i + 3 - 1] += CHSV( gHue + random8(64), 230, 20);
        ledsBottom[i + 3 + 1] += CHSV( gHue + random8(64), 230, 20);

        ledsBottom[-i + 99 - 3] += CHSV( gHue + random8(64), 230, 200);
        ledsBottom[-i + 99 - 3 + 1] += CHSV( gHue + random8(64), 230, 20);
        ledsBottom[-i + 99 - 3 - 1] += CHSV( gHue + random8(64), 230, 20);
    }
}

SimplePatternList gPatterns = {confetti}; //sinelon


// Send MIDI note ON
void midiNoteOn(byte note, byte midiVelocity)
{
  Serial.write(NOTE_ON_CMD);
  Serial.write(note);
  Serial.write(midiVelocity);
}

//Send MIDI note OFF
void midiNoteOff(byte note, byte midiVelocity)
{
  Serial.write(NOTE_OFF_CMD);
  Serial.write(note);
  Serial.write(midiVelocity);
}

// Send MIDI CC
void midiCC(byte cc, byte midiValue)
{
  Serial.write(CC_CMD);
  Serial.write(cc);
  Serial.write(midiValue);
}

void updateButtons() {

  for (int i = 0; i < NUM_BUTTONS; i++) {
      int state = Buttons[i]->getButtonState();
      
      //  Button is pressed     
      if (state == 0) 
      {
         midiNoteOn(Buttons[i]->Note,127);
      }
  
      //  Button is not pressed
      if (state == 1) 
  	  {
  		midiNoteOff(Buttons[i]->Note,0); 
      }
  }
}

void updateKnobs() {

  for (int i = 0; i < NUM_KNOBS; i++) {
      int state = Knobs[i]->getKnobState();

      midiCC(Knobs[i]->CC, map(state, 0, 1024, 0, 127));
      
  }
}


// void RGB_color(int red_light_value, int green_light_value, int blue_light_value)
//  {
//   analogWrite(red_light_pin, red_light_value);
//   analogWrite(green_light_pin, green_light_value);
//   analogWrite(blue_light_pin, blue_light_value);
// }



void setup() {
    Serial.begin(115200);
    FastLED.addLeds<NEOPIXEL, LED_PIN_BOTTOM>(ledsBottom, NUM_LEDS_BOTTOM); // Bottom leds
    FastLED.addLeds<NEOPIXEL, LED_PIN_TOP>(ledsTop, NUM_LEDS_TOP);

    FastLED.setBrightness(MAX_BRIGHTNESS);
}

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

void nextPattern()
{
    // add one to the current pattern number, and wrap around at the end
    gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE(gPatterns);
}

void loop() {

     // Call the current pattern function once, updating the 'leds' array
    gPatterns[gCurrentPatternNumber]();

    // send the 'leds' array out to the actual LED strip
    FastLED.show();  
    // insert a delay to keep the framerate modest
    //FastLED.delay(1000/FRAMES_PER_SECOND); 

    // do some periodic updates
    EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
    updateButtons();
    updateKnobs();
}