#ifndef SEQUENCER_MEMORY_H
#define SEQUENCER_MEMORY_H

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"

#define MAX_MIDI_VALUE 127
#define INIT_MIDI_VALUE 80

typedef enum {
    trigger_playback,
    velocity
} step_parameter;

static void *create_memory_array(int *dimensions, int n);
uint8_t ***init_sequencer_memory(int number_of_tracks);
static void free_memory_array(void *array, int *dimensions, int n);
void deinit_sequencer_memory(uint8_t ***memory, int number_of_tracks);
void write_sequencer_memory(uint8_t ***memory, int track, int step, step_parameter parameter, uint8_t value);
uint8_t read_sequencer_memory(uint8_t ***memory, int track, int step, step_parameter parameter);

#endif