// Correctly reading all shift registers //Integrated FAST LED
// Outputting all keys as midi //Outputting all pots as midi
// Adds timeout after certain inactivity
// Adds control of mist relay and fan transistors

#include <Arduino.h>
#include <FastLED.h> 
#include "MIDIUSB.h" 
#include <elapsedMillis.h>

int analogInputs[4];
int analogInputsOld[4];
int midiButtons[3];
int midiButtonsOld[3];
elapsedMillis timeout;         // Globally scoped - see comment above
#define TIMEOUTDELAY 180000    // Delay before goes to sleep
#define TIMEOUTDELAYFANS 10000 // How much longer the fans stay on

// First led strip, 200 leds, 50 leds in 4 rows 
#define NUM_LEDS_TOP 200
// Second strip on bottom, 98 leds, 49 in 2 rows 
#define NUM_LEDS_BOTTOM 98

CRGB ledsTop[NUM_LEDS_TOP];
CRGB ledsBottom[NUM_LEDS_BOTTOM];

// The total number of keyboard keys to poll the shift registers 
#define NUMKEYS 88

// Width of pulse to trigger the shift register to read and latch. 
#define PULSE_WIDTH_USEC 5

// Optional delay between shift register reads. 
#define POLL_DELAY_MSEC 1

// States of each shift register input 
bool keyState [NUMKEYS];
bool oldKeyState[NUMKEYS];

int ploadPin = 9;        // Connects to Parallel load pin the 165
int clockEnablePin = 10; // Connects to Clock Enable pin the 165 
int dataPin = 12; // Connects to the Q7 pin the 165
int clockPin = 13;       // Connects to the Clock pin the 165

#define BRIGHTNESS 96 
#define FRAMES_PER_SECOND 120

void setup()
{
    Serial.begin(115200);
    // Define switch pins 
    pinMode(3, INPUT_PULLUP); 
    pinMode(4, INPUT_PULLUP); 
    pinMode(8, INPUT_PULLUP);
    pinMode(6, OUTPUT); // Mist relay 
    pinMode(2,OUTPUT); //Fan transistor
    digitalWrite(2, LOW);
    digitalWrite(6, LOW);
    // Initialize our digital pins... 
    pinMode(ploadPin, OUTPUT); 
    pinMode(clockEnablePin, OUTPUT); 
    pinMode(clockPin, OUTPUT); 
    pinMode(dataPin, INPUT);
    FastLED.addLeds<NEOPIXEL, 7>(ledsBottom, NUM_LEDS_BOTTOM); // Bottom leds
    FastLED.addLeds<NEOPIXEL, 5>(ledsTop, NUM_LEDS_TOP);       // Top leds 
    //FastLED.setBrightness(10);
    digitalWrite(clockPin, LOW);
    digitalWrite(ploadPin, HIGH);
    // Parallel load the registers parallel_load();
    // initial read pots

    analogInputs[0] = analogRead(1);
    analogInputs[1] = analogRead(2);
    analogInputs[2] = analogRead(3);
    analogInputs[3] = analogRead(4);
    // read buttons
    midiButtons[0] = digitalRead(3);
    midiButtons[1] = digitalRead(4);
    midiButtons[2] = digitalRead(8);
}
// List of patterns to cycle through. Each is defined as a separate function below. 
typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = {confetti, sinelon};

uint8_t gHue = 0;                  // rotating "base color" used by many of the patterns
uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current

void noteOn(byte channel, byte pitch, byte velocity) {
    midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};
    MidiUSB.sendMIDI(noteOn);
    timeout = 0; // Reset timer when input detected gCurrentPatternNumber = 0; //set pattern to spots
}
void noteOff(byte channel, byte pitch, byte velocity) {
    midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity};
    MidiUSB.sendMIDI(noteOff);
    timeout = 0; // Reset timer when input detected gCurrentPatternNumber = 0; //set pattern to spots
}

void controlChange(byte channel, byte control, byte value) {
    midiEventPacket_t event = {0x0B, 0xB0 | channel, control, value};
    MidiUSB.sendMIDI(event);
    // timeout = 0; //Reset timer when input detected
}

void loop()
{

    digitalWrite(6, HIGH); // Misters digitalWrite(2, HIGH); //Fans
    if (timeout > TIMEOUTDELAY)
    {
        digitalWrite(6, LOW);
    }
    if (timeout > TIMEOUTDELAYFANS + TIMEOUTDELAY)
    {
        digitalWrite(2, LOW);
        gCurrentPatternNumber = 1;
    }
    // read pots
    analogInputs[0] = analogRead(1);
    analogInputs[1] = analogRead(2);
    analogInputs[2] = analogRead(3);
    analogInputs[3] = analogRead(4);
    // read buttons
    midiButtons[0] = digitalRead(3);
    midiButtons[1] = digitalRead(4);
    midiButtons[2] = digitalRead(8);
    // Call the current pattern function once, updating the 'leds' array gPatterns[gCurrentPatternNumber]();
    // send the 'leds' array out to the actual LED strip FastLED.show();
    // insert a delay to keep the framerate modest //FastLED.delay(1000/FRAMES_PER_SECOND);
    // do some periodic updates
    EVERY_N_MILLISECONDS(20) { gHue++; } // slowly cycle the "base color" through the rainbow
    // EVERY_N_SECONDS( 10 ) { nextPattern(); } // change patterns periodically
    // Parallel load the registers parallel_load();
    // Read the registers read_shift_regs();

    /*
    //If there was a change in state, display which ones changed.

    for(int i = 0; i < NUMKEYS; i++) {
    if(keyState[i] != oldKeyState[i]) {
    display_pin_values(); }
    } */

    delay(POLL_DELAY_MSEC);
    // Send MIDI signals
    for (int i = 0; i < NUMKEYS; i++)
    {
        if (keyState[i] == 1 && oldKeyState[i] == 0)
        {
            noteOn(0, 87 - i, 64); // Set note to send and flip key order 
            /*
            Serial.print("Button ");
            Serial.print(i);
            Serial.print(" is on.");
            Serial.print("\r\n");
            */
            MidiUSB.flush();
        }
        else if (keyState[i] == 0 && oldKeyState[i] == 1)
        {
            noteOff(0, 87 - i, 64); // Set note to send and flip key oder 
            /*
            Serial.print("Button ");
            Serial.print(i);
            Serial.print(" is off.");
            Serial.print("\r\n");
            */
            MidiUSB.flush();
        }
    }
    // Set old values to new
    for (int i = 0; i < NUMKEYS; i++) {
        oldKeyState[i] = keyState[i];
    }
    // If there's a change in pot, send midi 
    if(analogInputsOld[0] != analogInputs[0]) {
    // Serial.print(analogInputs [0]);
    // Serial.print("\r\n");
    controlChange(0, 16, map(analogInputs[0], 0, 1024, 0, 126)); // Set the value of controller 10 on channel 0 to 65 
    MidiUSB.flush();
    }

    if (analogInputsOld[1] != analogInputs[1])
    {
        // Serial.print(analogInputs [1]);
        // Serial.print("\r\n");
        controlChange(0, 17, map(analogInputs[1], 0, 1024, 0, 126)); // Set the value of controller 10 on channel 0 to 65 
        MidiUSB.flush();
    }
    if (analogInputsOld[2] != analogInputs[2])
    {
        // Serial.print(analogInputs [2]);
        // Serial.print("\r\n");
        controlChange(0, 18, map(analogInputs[2], 0, 1024, 0, 126)); // Set the value of controller 10 on channel 0 to 65
        MidiUSB.flush();
    }
    if (analogInputsOld[3] != analogInputs[3])
    {
        // Serial.print(analogInputs [3]);
        // Serial.print("\r\n");
        controlChange(0, 19, map(analogInputs[3], 0, 1024, 0, 126)); // Set the value of controller 10 on channel 0 to 65 
        MidiUSB.flush();
    }
    // if there's a change in button, send midi change 
    if(midiButtons[0] != midiButtonsOld[0])
    {
        controlChange(1, 21, map(midiButtons[0], 0, 1, 0, 127)); // Set the value of controller 10 on channel 0 to 65

        Serial.print(map(midiButtons[0], 0, 1, 0, 127));
        MidiUSB.flush();
    }
    if (midiButtons[1] != midiButtonsOld[1])
    {
        controlChange(1, 22, map(midiButtons[1], 0, 1, 0, 127)); // Set the value of controller 10 on channel 0 to 65
        Serial.print(map(midiButtons[1], 0, 1, 0, 127));
        MidiUSB.flush();
    }
    if (midiButtons[2] != midiButtonsOld[2])
    {
        controlChange(0, 23, map(midiButtons[2], 0, 1, 0, 127)); // Set the value of controller 10 on channel 0 to 65
        Serial.print(map(midiButtons[2], 0, 1, 0, 127));
        MidiUSB.flush();
    }
// Set old analog values 
analogInputsOld[0] = analogInputs[0]; 
analogInputsOld[1] = analogInputs[1]; 
analogInputsOld[2] = analogInputs[2]; 
analogInputsOld[3] = analogInputs[3];
midiButtonsOld[0] = midiButtons[0];
midiButtonsOld[1] = midiButtons[1];
midiButtonsOld[2] = midiButtons[2];
// Set LEDS based on key presses 
for(int i = 0; i < 44; i++)
{
    if (keyState[i])
    {
        ledsTop[i + 3] += CHSV(gHue + random8(64), 230, 200);
        ledsTop[i + 3 - 1] += CHSV(gHue + random8(64), 230, 20);
        ledsTop[i + 3 + 1] += CHSV(gHue + random8(64), 230, 20);
        ledsTop[-i + 99 - 3] += CHSV(gHue + random8(64), 230, 200);
        ledsTop[-i + 99 - 3 + 1] += CHSV(gHue + random8(64), 230, 20);
        ledsTop[-i + 99 - 3 - 1] += CHSV(gHue + random8(64), 230, 20);
        ///*

        ledsTop[i + 100 + 3] += CHSV(gHue + random8(64), 230, 200);
        ledsTop[i + 100 + 3 + 1] += CHSV(gHue + random8(64), 230, 20);
        ledsTop[i + 100 + 3 - 1] += CHSV(gHue + random8(64), 230, 20);
        ledsTop[-i + 199 - 3] += CHSV(gHue + random8(64), 230, 200);
        ledsTop[-i + 199 - 3 + 1] += CHSV(gHue + random8(64), 230, 20);
        ledsTop[-i + 199 - 3 - 1] += CHSV(gHue + random8(64), 230, 20); //*/
        // Flash Bottom
        ledsBottom[i + 3] += CHSV(gHue + random8(64), 230, 200);
        ledsBottom[i + 3 - 1] += CHSV(gHue + random8(64), 230, 20);
        ledsBottom[i + 3 + 1] += CHSV(gHue + random8(64), 230, 20);
        ledsBottom[-i + 99 - 3] += CHSV(gHue + random8(64), 230, 200);
        ledsBottom[-i + 99 - 3 + 1] += CHSV(gHue + random8(64), 230, 20);
        ledsBottom[-i + 99 - 3 - 1] += CHSV(gHue + random8(64), 230, 20);
    }

    if (keyState[i + 44])
    {
        ///*
        ledsTop[i + 3] += CHSV(gHue + random8(64), 230, 200);
        ledsTop[i + 3 - 1] += CHSV(gHue + random8(64), 230, 20);
        ledsTop[i + 3 + 1] += CHSV(gHue + random8(64), 230, 20);
        ledsTop[-i + 99 - 3] += CHSV(gHue + random8(64), 230, 200);
        ledsTop[-i + 99 - 3 + 1] += CHSV(gHue + random8(64), 230, 20);
        ledsTop[-i + 99 - 3 - 1] += CHSV(gHue + random8(64), 230, 20); //*/
        ledsTop[i + 100 + 3] += CHSV(gHue + random8(64), 230, 200);
        ledsTop[i + 100 + 3 + 1] += CHSV(gHue + random8(64), 230, 20);
        ledsTop[i + 100 + 3 - 1] += CHSV(gHue + random8(64), 230, 20);
        ledsTop[-i + 199 - 3] += CHSV(gHue + random8(64), 230, 200);
        ledsTop[-i + 199 - 3 + 1] += CHSV(gHue + random8(64), 230, 20);
        ledsTop[-i + 199 - 3 - 1] += CHSV(gHue + random8(64), 230, 20);
        // Flash Bottom
        ledsBottom[i + 3] += CHSV(gHue + random8(64), 230, 200);
        ledsBottom[i + 3 - 1] += CHSV(gHue + random8(64), 230, 20);

        ledsBottom[i + 3 + 1] += CHSV(gHue + random8(64), 230, 20);
        ledsBottom[-i + 99 - 3] += CHSV(gHue + random8(64), 230, 200);
        ledsBottom[-i + 99 - 3 + 1] += CHSV(gHue + random8(64), 230, 20);
        ledsBottom[-i + 99 - 3 - 1] += CHSV(gHue + random8(64), 230, 20);
    }
}
}
void parallel_load()
{
    // Trigger a parallel Load to latch the state of the register inputs 
    digitalWrite(clockEnablePin, HIGH);
    digitalWrite(ploadPin, LOW);
    delayMicroseconds(PULSE_WIDTH_USEC);
    digitalWrite(ploadPin, HIGH);
    digitalWrite(clockEnablePin, LOW);
}
void read_shift_regs()
{
    // Loop to read each input from registers 
    for(int i = 0; i < NUMKEYS; i++)
    {
        // Set state of current key 
        keyState[i] = digitalRead(dataPin);
        //  Pulse the Clock (rising edge shifts the next bit). 
        digitalWrite(clockPin, HIGH); 
        delayMicroseconds(PULSE_WIDTH_USEC); 
        digitalWrite(clockPin, LOW);
    }
}
/* Dump the list of zones along with their current status. */
void display_pin_values()
{
    for (int i = 0; i < NUMKEYS; i++)
    {

        if (keyState[i] && keyState[i] == 1)
        {
            Serial.print("Button");
            Serial.print(i);
            Serial.print(" ");
            // ledsTop[i] += CHSV( gHue + random8(64), 230, 200);
        }
    }
    Serial.print("\r\n");
}

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

void nextPattern()
{
    // add one to the current pattern number, and wrap around at the end
    gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE(gPatterns);
}

void confetti()
{
    // random colored speckles that blink in and fade smoothly 
    fadeToBlackBy( ledsTop, NUM_LEDS_TOP, 30); 
    fadeToBlackBy( ledsBottom, NUM_LEDS_BOTTOM, 20);
    int pos = random16(NUM_LEDS_TOP);
    ledsTop[pos] += CHSV(gHue + random8(64), 200, 150);
    // ledsBottom[pos] += CHSV( gHue + random8(64), 200, 150); 
}

void sinelon()
{
    // a colored dot sweeping back and forth, with fading trails 
    fadeToBlackBy( ledsTop, NUM_LEDS_TOP, 20);
    int pos = beatsin16(13, 0, NUM_LEDS_TOP - 1);
    ledsTop[pos] += CHSV(gHue, 255, 192);
}