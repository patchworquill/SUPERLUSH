#include <Arduino.h>
#include <FastLED.h>
#include <elapsedMillis.h>

#define NUM_POTS 6
int analogInputs[NUM_POTS];
int analogInputsPrev[NUM_POTS];

#define NUM_BUTTONS 10
int buttons[NUM_BUTTONS];
int buttonsPrev[NUM_BUTTONS];

elapsedMillis timeout;  // Globally scoped - see comment above

#define TIMEOUTDELAY 180000    // Delay before goes to sleep
#define TIMEOUTDELAYFANS 10000 // How much longer the fans stay on

// First led strip, 200 leds, 50 leds in 4 rows 
#define NUM_LEDS_TOP 100 // 98
// Second strip on bottom, 98 leds, 49 in 2 rows 
#define NUM_LEDS_BOTTOM 80

CRGB ledsTop[NUM_LEDS_TOP];
CRGB ledsBottom[NUM_LEDS_BOTTOM];

/************** PINOUT ******************
 *  ----- INPUTS -------
// A0-A5 | POT

// 2-5 MICRO SWITCHES - Joystick
// 8-13 MICRO SWITCHES
    // On until button is let go

// INPUTS
// 6 | LEDS_BOTTOM
// 7 | LEDS_TOP

******************************************/

#define J_UP 3
#define J_LEFT 2
#define J_DOWN 12
#define J_RIGHT 4

#define PB_1 5
#define PB_2 8
#define PB_3 9
#define PB_4 10
#define PB_5 11
#define PB_6 13

#define BRIGHTNESS          255
#define FRAMES_PER_SECOND  120

void setup()
{
    
    Serial.begin(115200);
    Serial.println("Initializing.");
    // Define switch pins 
    
    // Joystick
    pinMode(J_LEFT, INPUT_PULLUP); // LEFT
    pinMode(J_UP, INPUT_PULLUP); // UP
    pinMode(J_RIGHT, INPUT_PULLUP); // RIGHT
    pinMode(PB1, INPUT_PULLUP); 

    // Button Pad
    pinMode(PB_2, INPUT_PULLUP); 
    pinMode(PB_3, INPUT_PULLUP); 
    pinMode(PB_4, INPUT_PULLUP); 
    pinMode(PB_5, INPUT_PULLUP);
    pinMode(J_DOWN, INPUT_PULLUP); // DOWN
    pinMode(PB_6, INPUT_PULLUP);

    FastLED.addLeds<NEOPIXEL, 6>(ledsBottom, NUM_LEDS_BOTTOM); // Bottom leds
    FastLED.addLeds<NEOPIXEL, 7>(ledsTop, NUM_LEDS_TOP);
    
    // Read some initial values?
    // TODO: why is this here?
    analogInputs[0] = analogRead(1);
    analogInputs[1] = analogRead(2);
    analogInputs[2] = analogRead(3);
    analogInputs[3] = analogRead(4);
    analogInputs[4] = analogRead(5);
    analogInputs[5] = analogRead(6);
    // read buttons
    buttons[0] = digitalRead(PB_1);
    buttons[1] = digitalRead(PB_2);
    buttons[2] = digitalRead(PB_3);       
    buttons[3] = digitalRead(PB_4); 
    buttons[4] = digitalRead(PB_5); 
    buttons[5] = digitalRead(PB_6);
    // read joystick

    Serial.println("Initialization complete.");
}

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

// void noteOn(byte channel, byte pitch, byte velocity) {
//     midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};
//     MidiUSB.sendMIDI(noteOn);
//     timeout = 0; // Reset timer when input detected gCurrentPatternNumber = 0; //set pattern to spots
// }
// void noteOff(byte channel, byte pitch, byte velocity) {
//     midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity};
//     MidiUSB.sendMIDI(noteOff);
//     timeout = 0; // Reset timer when input detected gCurrentPatternNumber = 0; //set pattern to spots
// }

// void controlChange(byte channel, byte control, byte value) {
//     midiEventPacket_t event = {0x0B, 0xB0 | channel, control, value};
//     MidiUSB.sendMIDI(event);
//     // timeout = 0; //Reset timer when input detected
// }

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

void nextPattern()
{
    // add one to the current pattern number, and wrap around at the end
    gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE(gPatterns);
}

void loop() {
    // read pots
    // for (int i=0; i<analogInputs; i++){
    // Serial.println("Read line.");
    analogInputs[0] = analogRead(1);
    analogInputs[1] = analogRead(2);
    analogInputs[2] = analogRead(3);
    analogInputs[3] = analogRead(4);
    analogInputs[4] = analogRead(5);
    analogInputs[5] = analogRead(6);

    // read buttons
    buttons[0] = digitalRead(PB_1);
    buttons[1] = digitalRead(PB_2);
    buttons[2] = digitalRead(PB_3);       
    buttons[3] = digitalRead(PB_4); 
    buttons[4] = digitalRead(PB_5); 
    buttons[5] = digitalRead(PB_6);


    // If there's a change in pot, send midi 
    for (int i=0; i < NUM_ANALOG_INPUTS; i++){
        if(analogInputsPrev[i] != analogInputs[i]) {
            Serial.print("KNOB: ");
            Serial.print(i);
            Serial.print(analogInputs[i]);
            Serial.print("\r\n");
            flash_LEDs();
            // controlChange(0, 16, map(analogInputs[0], 0, 1024, 0, 126)); // Set the value of controller 10 on channel 0 to 65 
            // MidiUSB.flush(); 
        }
        
    }

    // TODO: use a less hack method for this. Could create less responsiveness
    for (int i=0; i < NUM_BUTTONS; i++){
        if(buttonsPrev[i] != buttons[i]){
            Serial.print("BUTTON ");
            Serial.print(i);
            Serial.print(buttons[i]);
            Serial.print("\r\n");
            flash_LEDs();
            
        }
    }

    //Set old analog values
    analogInputsPrev[0] = analogInputs[0];
    analogInputsPrev[1] = analogInputs[1];
    analogInputsPrev[2] = analogInputs[2];
    analogInputsPrev[3] = analogInputs[3];
    analogInputsPrev[4] = analogInputs[4];
    analogInputsPrev[5] = analogInputs[5];

    buttonsPrev[0] = buttons[0];
    buttonsPrev[1] = buttons[1];
    buttonsPrev[2] = buttons[2];
    buttonsPrev[3] = buttons[3];
    buttonsPrev[4] = buttons[4];
    buttonsPrev[5] = buttons[5];

    delay(1000);

}