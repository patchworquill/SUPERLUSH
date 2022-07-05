I was using shift registers before to convert analog signal to digital to afford more lines of communication through the limited series of ports on the arduino

INS
6 analog INS and need sensor values read
-> sent to midi
Same for 10 digital ins (all push Buttons)

OUTS
digital pin 6 and 7 to addressable LEDs

---

## Log
- [x] Get the Arduino Uno rewired
- [x] Get LEDs out
- [x] Get the Knob reading
- [x] Get the Buttons reading
  - [ ] Get the Joystick Reading
- [x] Get the Debounce function working
  - [ ] Get it working perfectly
- [x] Get the MIDI sending through Hairless -> loopMIDI
  - https://create.arduino.cc/projecthub/sadreactonly/unopad-arduino-midi-controller-with-ableton-67ea75
- [ ] Get the Buttons sending MIDI Notes
- [ ] Get the Knobs Sending MIDI Notes / CCs

---

## Software
- Primarily following [this tutorial](https://create.arduino.cc/projecthub/sadreactonly/unopad-arduino-midi-controller-with-ableton-67ea75)
- Using Hairless Serial to MIDI Bridge
- Using loopMIDI for Virtual MIDI Loopback