#include "sequencer_memory.h"

#define NUMBER_OF_STEPS 16
#define NUMBER_OF_PARAMETERS_PER_STEP 2

static void *create_memory_array(int *dimensions, int n) {
    if (n == 0) {
        return NULL;
    }

    int size = dimensions[0];
    void **array = malloc(size * sizeof(void *));

    if (n == 1) {
        for (int i = 0; i < size; i++) {
            array[i] = malloc(sizeof(uint8_t)); // Allocate space for integers
        }
    } 
    else {
        for (int i = 0; i < size; i++) {
            array[i] = create_memory_array(dimensions + 1, n - 1);
        }
    }

    return array;
}

uint8_t ***init_sequencer_memory(int number_of_tracks) {
    int dimensions[] = {number_of_tracks, NUMBER_OF_STEPS, NUMBER_OF_PARAMETERS_PER_STEP};
    int n = sizeof(dimensions) / sizeof(dimensions[0]);

    // Create the 3-dimensional array
    void *array = create_memory_array(dimensions, n);
    uint8_t ***sequencer_memory = (uint8_t ***)array;

    // Initialize memory values
    for (int i = 0; i < number_of_tracks; i++)
        for (int j = 0; j < NUMBER_OF_STEPS; j++)
            for(int k = 0; k < NUMBER_OF_PARAMETERS_PER_STEP; k++) {
                if (k == trigger_playback) sequencer_memory[i][j][k] = 0;
                else sequencer_memory[i][j][k] = INIT_MIDI_VALUE;
            }
        
    return sequencer_memory;
}

static void free_memory_array(void *array, int *dimensions, int n) {
    if (n == 0 || array == NULL) {
        return;
    }

    int size = dimensions[0];
    void **arr = (void **)array;

    if (n == 1) {
        for (int i = 0; i < size; i++) {
            free(arr[i]);
        }
    } 
    else {
        for (int i = 0; i < size; i++) {
            free_memory_array(arr[i], dimensions + 1, n - 1);
        }
    }

    free(arr);
}

void deinit_sequencer_memory(uint8_t ***memory, int number_of_tracks) {
    void *array = (void *)memory;
    int dimensions[] = {number_of_tracks, NUMBER_OF_STEPS, NUMBER_OF_PARAMETERS_PER_STEP};
    int n = sizeof(dimensions) / sizeof(dimensions[0]);
    free_memory_array(array, dimensions, n);
}

void write_sequencer_memory(uint8_t ***memory, int track, int step, step_parameter parameter, uint8_t value) {
    switch (parameter) {
        case trigger_playback:
            memory[track][step][parameter] = !memory[track][step][parameter];
            break;
        case velocity:
            memory[track][step][parameter] = value * MAX_MIDI_VALUE / (1 << 4);
            break;
    }
}

uint8_t read_sequencer_memory(uint8_t ***memory, int track, int step, step_parameter parameter) {
     return memory[track][step][parameter];
}
