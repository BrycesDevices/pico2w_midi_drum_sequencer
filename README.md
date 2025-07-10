MIDI DRUM SEQUENCER PROTOTYPE DESIGN

This repository contains the project files for a MIDI drum sequencer prototype design implemented on a Raspberry Pi Pico 2 W microcontroller.

A demo of the prototype can be found here: https://youtu.be/M-kd4VmO2eI

The design is programmed in C and uses API provided by the Pico C/C++ SDK, FreeRTOS, and TinyUSB libraries.

Some additional library files and elements of code contained in this design have been adapted from the following repositories:
 - https://github.com/raspberrypi/pico-examples/blob/master/dma/channel_irq/channel_irq.c
 - https://github.com/raspberrypi/pico-examples/blob/master/freertos/hello_freertos/hello_freertos.c
 - https://github.com/hathach/tinyusb/blob/master/examples/device/midi_test/src/main.c
 - https://github.com/sztvka/pico-sh1106-c/tree/main

PROJECT DESCRIPTION:

The design is a 16 step sequencer with four programmable tracks. Each of step of the four 16 step tracks are represented by a rectangle on a 1.3" OLED display in a matrix. 
Only one track is activated at a time for step editing and the active track is indicated at the top of the display by a number corresponding to a row in the matrix in decending order. 
A track select button is used to select the active track by cycling through the track numbers. Each track is assigned to send a "note on" midi message during playback which triggers a 
MIDI drum sound to play in a Digital Audio Workspace (DAW). For example, the note C1 is commonly mapped to the kick drum sound on the MIDI drum instruments in Logic Pro X, which is the 
DAW I am using with this project. Tracks 1-4 are assigned to trigger the hi-hat, cowbell, snare, and kick drum, respectively. 

Eight step select buttons are used to activate steps in the sequence or input per step parameter values. The positions of the step select buttons correspond to the step positions in 
the active track. For example, pressing the 4th button in the row of step select butons will activate the 4th step in the sequence. When a step is activated, the empty rectangle on the 
display corrseponding to that step becomes a solid rectangle. A step transpose button is used to access the upper eight steps and parameter values. A left and right arrow on the display 
are used to indicate which half of the sequence is being edited by highlighting arrow corresponding to the active region. Additionally, the last step select button pressed is indicated 
at the top right of the display.

As previously mentioned, the design also supports per step parameter editing. A parameter select button is used to cycle through the available per step parameters with the active parameter 
indicated on the display. Currently, there are only two parameters implemented in the desgin, however the code is structured to easily support future additions. The default parameter is the 
"set" parameter which is used for basic sequence programming as previously described. The "vel" parameter is used to program the MIDI velocity of the note played at that step. This feature 
can be used to create accents for dynamic sequence programming. When the "vel" parameter is selected, first select the step you would like to edit the same as you would to set the step for 
playback. The characters "vel" will become highlighted on the display indicating that the sequencer is waiting to recieve another input for the desired velocity value. This is where the step 
indicator on the display is helpful to remember which step is being edited. Next, enter a velocity value of 1-16 by pressing the corresponding step select button (note: the step transpose 
button can still be used to access the upper 8 values). The new velcocity value for that step is now saved and the "vel" is no longer highlighted.

The design is actively listening for MIDI system real time messages to receive "Start" and "Timing Clock" messages from Logic Pro X for the purposes of syncronizing and corridinating playback 
of the sequence to the tempo of the recording project in the DAW. Additionally, a Start/Stop button sends MIDI "note on" messages on a second MIDI channel which are mapped to global start and 
stop functions within Logic Pro X using the software's built in controller assignment feature.

Finally, there are four control knobs which are implemented by four potentiometers connected to individual channels of an external 8-channel MUX IC to save on ADC pins on the microcontroller. 
The first three control knobs are assigned to control various MIDI controls by sending MIDI CC messages, such as MIDI volume, panning, and modulation wheel. The fourth control knob adjusts the 
amount of swing in the sequence playback by adding a short delay to every other step in the sequence.

BREADBOARD CONFIGURATION:

<img width="1502" height="827" alt="good" src="https://github.com/user-attachments/assets/23c4acd3-a5ab-4489-aa7d-54abc8088222" />


