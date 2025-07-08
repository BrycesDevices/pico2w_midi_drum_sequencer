#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/cyw43_arch.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/dma.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "bsp/board_api.h"
#include "tusb.h"

#include "sh1106_i2c.h"
#include "font_inconsolata.h"

#include "sequencer_memory.h"

// Delay between led blinking in ms
#define LED_DELAY_MS 500

// Reference for GPIOs
#define FIRST_BUTTON 2
#define NUMBER_OF_STEP_BUTTONS 8
#define NUMBER_OF_UTILIY_BUTTONS 4
#define FIRST_MUX 14
#define NUMBER_OF_MUX_SELECT 2

// Button debounce delay time in ms
#define DEBOUNCE_MS 30

// For ADC/DMA configuration
#define NUM_KNOBS 4
#define SAMPLES_PER_CHANNEL 32

// MIDI CC numbers
#define MIDI_VOLUME 0x07
#define PAN_POSITION 0x0A
#define MOD_WHEEL 0x01

// For MIDI
#define MIDI_CHANNEL_1 0
#define MIDI_CHANNEL_2 1
#define MIDI_LENGTH 3
#define ZERO_VELOCITY 0
#define NOTE_ON_PREFIX 0x90
#define NOTE_OFF_PREFIX 0x80
#define CC_PREFIX 0xB0
#define START_MESSAGE 0x50
#define STOP_MESSAGE 0x5D
#define START 0xFF
#define CLOCK_PULSE 0xFE

// MIDI clock pulses per note length
#define WHOLE_NOTE 96
#define HALF_NOTE 48
#define DOTTED_QUARTER_NOTE 36
#define QUARTER_NOTE 24
#define DOTTED_EIGHTH_NOTE 18
#define QUARTER_NOTE_TRIPLET 16
#define EIGHTH_NOTE 12
#define DOTTED_SIXTEENTH_NOTE 9
#define EIGHTH_NOTE_TRIPLET 8
#define SIXTEENTH_NOTE 6
#define SIXTEENTH_NOTE_TRIPLET 4

// MIDI drum instrument note values
#define KICK 36
#define SNARE 38
#define HIHAT 42
#define CLAP 39
#define COWBELL 56
#define RIMSHOT 40

// For OLED Display
#define DISPLAY_ADDRESS 0x3C
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#define I2C_SDA 16
#define I2C_SCL 17
#define I2C_BAUD 400000

// Task priority levels
#define INPUT_TASK_PRIORITY ( tskIDLE_PRIORITY + 4UL )
#define USBD_TASK_PRIORITY ( tskIDLE_PRIORITY + 3UL )
#define MIDI_TASK_PRIORITY ( tskIDLE_PRIORITY + 3UL )
#define DISPLAY_TASK_PRIORITY ( tskIDLE_PRIORITY + 2UL )
#define BLINK_TASK_PRIORITY ( tskIDLE_PRIORITY + 1UL )

// Stack sizes of our threads in words (4 bytes)
// Increase stack size when debug log is enabled
#define USBD_TASK_STACK_SIZE        (3*configMINIMAL_STACK_SIZE/2) * (CFG_TUSB_DEBUG ? 2 : 1)
#define DISPLAY_TASK_STACK_SIZE     configMINIMAL_STACK_SIZE
#define MIDI_TASK_STACK_SIZE        configMINIMAL_STACK_SIZE * 2
#define BLINK_TASK_STACK_SIZE       configMINIMAL_STACK_SIZE
#define BUTTON_TASK_STACK_SIZE      configMINIMAL_STACK_SIZE * 2
#define KNOBS_TASK_STACK_SIZE       configMINIMAL_STACK_SIZE * 4

typedef struct {
  bool step_transpose; // false = first 8 steps, true = last 8 steps
  int active_track;
  step_parameter active_parameter;
  uint8_t active_step;
  bool edit_step;
} user_interface_t;

// Function Prototypes
void usb_device_task(__unused void *param);

void led_blinking_task(__unused void *params);

void midi_clock_task(__unused void *param);

void midi_playback_task(void *param);

void button_task(void *param);
void process_button_response(uint8_t ***memory, uint gpio, user_interface_t *ui);
void button_pressed_callback(uint gpio, uint32_t events);

void knobs_task(__unused void *param);
void dma_handler();
void mux_select_decoder(int select);
uint8_t assign_knob_midi_cc(int knob);

void display_task(void *param);
void setup_i2c(uint sda, uint scl, uint baud);
void update_display(sh1106_t *display, uint8_t ***memory, user_interface_t ui);


//--------------------------------------------------------------------+
// MAIN
//--------------------------------------------------------------------+

TaskHandle_t usbd_task_handle;
TaskHandle_t button_task_handle;
TaskHandle_t midi_clock_task_handle;
TaskHandle_t midi_playback_task_handle;
TaskHandle_t knobs_task_handle;
TaskHandle_t display_task_handle;
QueueHandle_t button_queue;
QueueHandle_t display_queue;
QueueHandle_t midi_queue;
SemaphoreHandle_t memory_mutex;

int main( void ) {
  board_init();

  // Allocate memory for sequencer
  const int number_of_tracks = 4;
  uint8_t ***sequencer_memory = init_sequencer_memory(number_of_tracks);

  // Create mutex for memory access
  memory_mutex = xSemaphoreCreateMutex();

  // Create queue for button ISR handler
  button_queue = xQueueCreate(10, sizeof(uint));
  display_queue = xQueueCreate(10, sizeof(user_interface_t));
  midi_queue = xQueueCreate(10, sizeof(uint8_t));

  // Configure 12 push button inputs
  for (int button_gpio = FIRST_BUTTON; button_gpio < FIRST_BUTTON + NUMBER_OF_STEP_BUTTONS + NUMBER_OF_UTILIY_BUTTONS; button_gpio++) {
    gpio_init(button_gpio);
    gpio_set_dir(button_gpio, GPIO_IN);
    gpio_pull_up(button_gpio);
    gpio_set_irq_enabled_with_callback(button_gpio, GPIO_IRQ_EDGE_FALL, true, &button_pressed_callback);
  }

  // Configure 2 or 3 mux select pins for knob mux
  for (int mux_select_gpio = FIRST_MUX; mux_select_gpio < FIRST_MUX + NUMBER_OF_MUX_SELECT; mux_select_gpio++) {
    gpio_init(mux_select_gpio);
    gpio_set_dir(mux_select_gpio, GPIO_OUT);
  }

  // Create tasks
  xTaskCreate(display_task, "displayThread", DISPLAY_TASK_STACK_SIZE, (void *) sequencer_memory, DISPLAY_TASK_PRIORITY, &display_task_handle);
  xTaskCreate(button_task, "buttonThread", BUTTON_TASK_STACK_SIZE, (void *) sequencer_memory, INPUT_TASK_PRIORITY, &button_task_handle);
  xTaskCreate(knobs_task, "knobsThread", KNOBS_TASK_STACK_SIZE, NULL, INPUT_TASK_PRIORITY, &knobs_task_handle);
  xTaskCreate(usb_device_task, "usbdThread", USBD_TASK_STACK_SIZE, NULL, USBD_TASK_PRIORITY, &usbd_task_handle);
  xTaskCreate(midi_clock_task, "midiClockThread", MIDI_TASK_STACK_SIZE, NULL, MIDI_TASK_PRIORITY, &midi_clock_task_handle);
  xTaskCreate(midi_playback_task, "midiPlaybackThread", MIDI_TASK_STACK_SIZE, (void *) sequencer_memory, MIDI_TASK_PRIORITY, &midi_playback_task_handle);
  xTaskCreate(led_blinking_task, "blinkThread", BLINK_TASK_STACK_SIZE, NULL, BLINK_TASK_PRIORITY, NULL);

  // Bind any usbd related tasks to one core
  #if configUSE_CORE_AFFINITY && configNUMBER_OF_CORES > 1
    vTaskCoreAffinitySet(button_task_handle, 1);
    vTaskCoreAffinitySet(knobs_task_handle, 1);
    vTaskCoreAffinitySet(usbd_task_handle, 1);
    vTaskCoreAffinitySet(midi_clock_task_handle, 1);
    vTaskCoreAffinitySet(midi_playback_task_handle, 1);
  #endif

  // Start the tasks and timer running
  vTaskStartScheduler();

  return 0;
}

//--------------------------------------------------------------------+
// DISPLAY TASK
//--------------------------------------------------------------------+

void setup_i2c(uint sda, uint scl, uint baud) {
  i2c_init(i2c0, baud);
  gpio_set_function(sda, GPIO_FUNC_I2C);
  gpio_set_function(scl, GPIO_FUNC_I2C);
  gpio_pull_up(sda);
  gpio_pull_up(scl);
}

void update_display(sh1106_t *display, uint8_t ***memory, user_interface_t ui) {
  // Clear display
  SH1106_clear(display);
  
  char display_string[16];
  int edit_color;
  int half_color;

  // Update active user interface parameters
  sprintf(display_string, "Trk:%d Step:%d", ui.active_track + 1, ui.active_step + 1);
  SH1106_drawString(display, display_string, 0, 0, STANDARD_COLOR, inconsolata);
  strcpy(display_string, "Para:");
  SH1106_drawString(display, display_string, 0, 16, STANDARD_COLOR, inconsolata);

  // Change color to indicate step edit input value required
  if (ui.edit_step)
      edit_color = INVERTED_COLOR;
  else
    edit_color = STANDARD_COLOR;

  switch (ui.active_parameter) {
    case trigger_playback:
      strcpy(display_string, "set");
      break;
    case velocity:
      strcpy(display_string, "vel");
      break;
  }
  
  SH1106_drawString(display, display_string, 48, 16, edit_color, inconsolata);

  // Change color to indicate active half of sequencer input
  if (ui.step_transpose)
    half_color = STANDARD_COLOR;
  else
    half_color = INVERTED_COLOR;
  
  SH1106_drawString(display, "<", 108, 16, half_color, inconsolata);
  SH1106_drawString(display, ">", 118, 16, !half_color, inconsolata);

  // Create sequence blocks
  uint8_t x_position = 0;
  uint8_t y_position = 32;
  uint8_t step_width = 8;
  uint8_t step_height = 8;
  int parameter = trigger_playback;

  for (int track = 0; track < 4; track++) {
    for (int step = 0; step < 16; step++) {
      if (read_sequencer_memory(memory, track, step, parameter))
        SH1106_drawActiveStep(display, x_position, y_position, step_width, step_height);
      else
        SH1106_drawEmptyStep(display, x_position, y_position, step_width, step_height);
      x_position += 8;
    }
    x_position = 0;
    y_position = 32 + ((track + 1) * 8);
  }

  // Update display
  SH1106_draw(display);
}

void display_task(void *param) {
  uint8_t ***sequencer_memory = (uint8_t ***) param;
  sh1106_t oled_display;
  user_interface_t ui;

  // wait for ui initialization
  xQueueReceive(display_queue, &ui, portMAX_DELAY);

  setup_i2c(I2C_SDA, I2C_SCL, I2C_BAUD);
  SH1106_init(&oled_display, i2c0, DISPLAY_ADDRESS, DISPLAY_WIDTH, DISPLAY_HEIGHT);
  xSemaphoreTake(memory_mutex, portMAX_DELAY);
  update_display(&oled_display, sequencer_memory, ui);
  xSemaphoreGive(memory_mutex);

  while (1) {
    // Wait for notification to update display
    xQueueReceive(display_queue, &ui, portMAX_DELAY);

    // Update sequence display
    xSemaphoreTake(memory_mutex, portMAX_DELAY);
    update_display(&oled_display, sequencer_memory, ui);
    xSemaphoreGive(memory_mutex);
  }
}

//--------------------------------------------------------------------+
// MIDI CLOCK TASK
//--------------------------------------------------------------------+

void midi_clock_task(__unused void* param) {
  uint8_t rx_buf[4];
  uint8_t sequencer_control;

  while (1) {
    // check for incoming midi messages
    while (tud_midi_available()) {
      tud_midi_packet_read(rx_buf);
      for (int i = 0; i < sizeof(rx_buf); i++) {
        // Poll for System Real Time messages
        switch (rx_buf[i]) {
          case MIDI_STATUS_SYSREAL_TIMING_CLOCK:
            sequencer_control = CLOCK_PULSE;
            xQueueSendToBack(midi_queue, &sequencer_control, portMAX_DELAY);
            break;
          case MIDI_STATUS_SYSREAL_START:
            sequencer_control = START;
            xQueueSendToBack(midi_queue, &sequencer_control, portMAX_DELAY);
            break;
        }
      }
    }
  }
}

//--------------------------------------------------------------------+
// MIDI PLAYBACK TASK
//--------------------------------------------------------------------+

const uint8_t cable_num = 0; // MIDI jack associated with USB endpoint

void midi_playback_task(void *param) {
  uint8_t ***sequencer_memory = (uint8_t ***) param;
  uint8_t midi_control;
  uint32_t tick_count = 0;
  uint8_t step_position = 0;
  uint8_t midi_note;
  bool turn_off[4] = {false};
  uint8_t swing = 0;

  while (1) {
    // Block on the queue to wait for midi clock data to arrive
    xQueueReceive(midi_queue, &midi_control, portMAX_DELAY);

    if (midi_control == START) {
      // Reset to starting conditions
      tick_count = 0;
      step_position = 0;
      bool turn_off[4] = {false};
    }
    else if (midi_control == CLOCK_PULSE) {
      // Check for active step in programmed 16th note sequence
      // Delay every other 16th note to implement swing
      if (tick_count == 0 || tick_count == SIXTEENTH_NOTE + swing) {

        xSemaphoreTake(memory_mutex, portMAX_DELAY);
        for (int track = 0; track < 4; track++) {

          // Assign each track to trigger a MIDI drum instrument
          switch (track) {
            case 0: // track 1
              midi_note = HIHAT;
              break;
            case 1: // track 2
              midi_note = COWBELL;
              break;
            case 2: // track 3
              midi_note = SNARE;
              break;
            case 3: // track 4
              midi_note = KICK;
              break;
          }

          // turn off previously triggered MIDI note
          if (turn_off[track]) {
            // Send Note Off MIDI message for previous note
            uint8_t note_off[MIDI_LENGTH] = { NOTE_OFF_PREFIX | MIDI_CHANNEL_1, midi_note, ZERO_VELOCITY};
            tud_midi_stream_write(cable_num, note_off, MIDI_LENGTH);
            // Clear turn off flag
            turn_off[track] = false;
          }

          if (read_sequencer_memory(sequencer_memory, track, step_position, trigger_playback)) {
            // Send Note On MIDI message 
            uint8_t note_on[MIDI_LENGTH] = { NOTE_ON_PREFIX | MIDI_CHANNEL_1, midi_note, read_sequencer_memory(sequencer_memory, track, step_position, velocity) };
            tud_midi_stream_write(cable_num, note_on, MIDI_LENGTH);
            // set flag to send Note Off message on the next step
            turn_off[track] = true;
          }
        }
        xSemaphoreGive(memory_mutex);

        // loop step position every 16 steps
        if (step_position == 15)
          step_position = 0;
        else 
          step_position++; 
      }
      // loop tick count every two 16th notes
      if (tick_count == (2 * SIXTEENTH_NOTE) - 1)
        tick_count = 0;
      else 
        tick_count++;
    }
    else {
      // update swing value
      swing = midi_control;
    }
  }
}

//--------------------------------------------------------------------+
// BUTTON TASK
//--------------------------------------------------------------------+

void button_pressed_callback(uint gpio, uint32_t events) {
  // disable IRQ 
  gpio_set_irq_enabled(gpio, GPIO_IRQ_EDGE_FALL, false);

  BaseType_t xHigherPriorityTaskWoken; 
  xHigherPriorityTaskWoken = pdFALSE; 
  xQueueSendToBackFromISR(button_queue, &gpio, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken); 
}

void process_button_response(uint8_t ***memory, uint gpio, user_interface_t *ui) {

  uint8_t value;
  uint8_t current_step = gpio - FIRST_BUTTON + (ui->step_transpose * 8);
  static bool start_stop = true;

  if (gpio < FIRST_BUTTON + NUMBER_OF_STEP_BUTTONS) {
    // trigger_playback parameter does not require value input
    if (ui->active_parameter != trigger_playback) {
      // select step to edit
      if (!ui->edit_step) {
        ui->edit_step = true;
        ui->active_step = current_step;
        return;
      }
      // select parameter value
      else {
        value = current_step;
        ui->edit_step = false;
      }
    }
    else {
      // Set playback of selected step in sequence
      ui->active_step = current_step;
      value = true;
    }
    // Update step memory
    xSemaphoreTake(memory_mutex, portMAX_DELAY);
    write_sequencer_memory(memory, ui->active_track, ui->active_step, ui->active_parameter, value);
    xSemaphoreGive(memory_mutex);
  }
  else {
    switch (gpio) {
      case FIRST_BUTTON + NUMBER_OF_STEP_BUTTONS:
        if (ui->edit_step) return; // must select step edit value before using other functions
        // Arm next Sequencer track (scroll down/loop back to the top)
        if (ui->active_track == 3)
          ui->active_track = 0;
        else
          ui->active_track++;
        break;
      case FIRST_BUTTON + NUMBER_OF_STEP_BUTTONS + 1:
        // Toggle between first and last 8 steps in the armed sequence
        ui->step_transpose = !ui->step_transpose;
        break;
      case FIRST_BUTTON + NUMBER_OF_STEP_BUTTONS + 2:
        if (ui->edit_step) return; // must select step edit value before using other functions
        // Cycle through per step parameters
        if (ui->active_parameter == velocity)
          ui->active_parameter = trigger_playback;
        else
          ui->active_parameter++;
        break;
      case FIRST_BUTTON + NUMBER_OF_STEP_BUTTONS + 3:
        // Send Start/Stop MIDI messages for sequence playback
        if (start_stop) {
          uint8_t midi_message[MIDI_LENGTH] = {NOTE_ON_PREFIX | MIDI_CHANNEL_2, START_MESSAGE, MAX_MIDI_VALUE};
          tud_midi_stream_write(cable_num, midi_message, MIDI_LENGTH);
          uint8_t sequencer_control = START;
          xQueueSendToBack(midi_queue, &sequencer_control, portMAX_DELAY);
        }
        else {
          uint8_t midi_message[MIDI_LENGTH] = {NOTE_ON_PREFIX | MIDI_CHANNEL_2, STOP_MESSAGE, MAX_MIDI_VALUE};
          tud_midi_stream_write(cable_num, midi_message, MIDI_LENGTH);
        }
        start_stop = !start_stop;
        break;
    }
  }
}

void button_task(void *param) {
  uint8_t ***sequencer_memory = (uint8_t ***) param;
  uint gpio;
  user_interface_t sequencer_ui;
  sequencer_ui.active_track = 0;
  sequencer_ui.active_step = 0;
  sequencer_ui.active_parameter = trigger_playback;
  sequencer_ui.step_transpose = false;
  sequencer_ui.edit_step = false;

  xQueueSendToBack(display_queue, &sequencer_ui, portMAX_DELAY);

  while(1) {
    // Block on the queue to wait for data to arrive
    xQueueReceive(button_queue, &gpio, portMAX_DELAY);

    // Implement debounce delay
    vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));

    if (!gpio_get(gpio)) {
    process_button_response(sequencer_memory, gpio, &sequencer_ui);
    xQueueSendToBack(display_queue, &sequencer_ui, portMAX_DELAY);
    }

    // re-enable IRQ
    gpio_set_irq_enabled(gpio, GPIO_IRQ_EDGE_FALL, true);
  }
}

//--------------------------------------------------------------------+
// KNOBS TASK
//--------------------------------------------------------------------+

volatile int dma_channel;

void dma_handler() {
  // Clear IRQ
  dma_hw->ints0 = 1u << dma_channel;

  // Notify task
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(knobs_task_handle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void mux_select_decoder(int select) {
  switch (select) {
    case 0:
      gpio_put(FIRST_MUX, false);
      gpio_put(FIRST_MUX + 1, false);
      //gpio_put(FIRST_MUX + 2, false);
      break;
    case 1:
      gpio_put(FIRST_MUX, true);
      gpio_put(FIRST_MUX + 1, false);
      //gpio_put(FIRST_MUX + 2, false);
      break;
    case 2:
      gpio_put(FIRST_MUX, false);
      gpio_put(FIRST_MUX + 1, true);
      //gpio_put(FIRST_MUX + 2, false);
      break;
    case 3:
      gpio_put(FIRST_MUX, true);
      gpio_put(FIRST_MUX + 1, true);
      //gpio_put(FIRST_MUX + 2, false);
      break;
    case 4:
      gpio_put(FIRST_MUX, false);
      gpio_put(FIRST_MUX + 1, false);
      //gpio_put(FIRST_MUX + 2, true);
      break;
    case 5:
      gpio_put(FIRST_MUX, true);
      gpio_put(FIRST_MUX + 1, false);
      //gpio_put(FIRST_MUX + 2, true);
      break;
    case 6:
      gpio_put(FIRST_MUX, false);
      gpio_put(FIRST_MUX + 1, true);
      //gpio_put(FIRST_MUX + 2, true);
      break;
    case 7:
      gpio_put(FIRST_MUX, true);
      gpio_put(FIRST_MUX + 1, true);
      //gpio_put(FIRST_MUX + 2, true);
      break;
  }
}

uint8_t assign_knob_midi_cc(int knob) {
  switch (knob) {
        case 0:
          return MIDI_VOLUME;
          break;
        case 1:
          return PAN_POSITION;
          break;
        case 2:
          return MOD_WHEEL;
          break;
      }
}

void knobs_task(__unused void *param) {
  adc_init();

  // Initialize ADC GPIO for knob
  const uint8_t adc0 = 26;
  adc_gpio_init(adc0);

  // Set up FIFO for DMA use
  const uint8_t fifo_samples = 1;
  adc_fifo_setup(true, true, fifo_samples, false, false);

  const uint8_t adc_input = 0;
  adc_select_input(adc_input);

  // Claim and configure DMA channel
  dma_channel = dma_claim_unused_channel(true);
  dma_channel_config dma_config = dma_channel_get_default_config(dma_channel);
  channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_16);
  channel_config_set_read_increment(&dma_config, false);
  channel_config_set_write_increment(&dma_config, true);
  channel_config_set_dreq(&dma_config, DREQ_ADC);

  // Set up DMA interrupt
  dma_channel_set_irq0_enabled(dma_channel, true);
  irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
  irq_set_enabled(DMA_IRQ_0, true);

  uint8_t midi_CC_number;
  uint16_t adc_buffers[NUM_KNOBS][SAMPLES_PER_CHANNEL];
  uint8_t previous_adc_value[NUM_KNOBS];

  const uint8_t change_threshold = 64;

  while (1) {
    for (int knob_select = 0; knob_select < NUM_KNOBS; knob_select++) {
      // Select mux channel for knob of interest
      mux_select_decoder(knob_select);

      // Wait for probagation delay through IC
      vTaskDelay(pdMS_TO_TICKS(1));

      // Clear FIFO before starting new sample
      adc_fifo_drain();

      // Start DMA
      dma_channel_configure(dma_channel, &dma_config, &adc_buffers[knob_select], &adc_hw->fifo, SAMPLES_PER_CHANNEL, true);

      // Start ADC
      adc_run(true);

      // Wait for DMA to complete (not blocking the CPU)
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

      // Stop ADC
      adc_run(false);

      if (knob_select < NUM_KNOBS - 1)
        // Assign knob MIDI control function
        midi_CC_number = assign_knob_midi_cc(knob_select);

      // Process samples
      for (int i = 0; i < SAMPLES_PER_CHANNEL; i++) {

        // Calculate absolute change in knob position
        uint8_t change = (adc_buffers[knob_select][i] > previous_adc_value[knob_select]) ? (adc_buffers[knob_select][i] - previous_adc_value[knob_select]) : (previous_adc_value[knob_select] - adc_buffers[knob_select][i]);
        
        if (change > change_threshold) {

          if (knob_select < NUM_KNOBS - 1) {
            // Calculate midi value from raw adc value read from buffer
            uint8_t midi_value = adc_buffers[knob_select][i] * MAX_MIDI_VALUE / (1 << 12);
            // Generate and send midi message
            uint8_t CC_message[MIDI_LENGTH] = { CC_PREFIX | MIDI_CHANNEL_1, midi_CC_number, midi_value};
            tud_midi_stream_write(cable_num, CC_message, MIDI_LENGTH);
          }
          else {
            // implement swing control on knob 4
            uint8_t swing_value = adc_buffers[knob_select][i] * SIXTEENTH_NOTE / (1 << 12);
            xQueueSendToBack(midi_queue, &swing_value, portMAX_DELAY);
          }
          previous_adc_value[knob_select] = adc_buffers[knob_select][i];
        }
      }
    }
    // Sample all knob channels 10 times every second
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

//--------------------------------------------------------------------+
// USB DEVICE DRIVER TASK
//--------------------------------------------------------------------+

void usb_device_task(__unused void *param) {
  // init device stack on configured roothub port
  tusb_rhport_init_t dev_init = {
    .role = TUSB_ROLE_DEVICE,
    .speed = TUSB_SPEED_AUTO
  };
  tusb_init(BOARD_TUD_RHPORT, &dev_init);

  if (board_init_after_tusb) {
    board_init_after_tusb();
  }

  // RTOS forever loop
  while (1) {
    // put this thread to waiting state until there is new events
    tud_task();
  }
}

//--------------------------------------------------------------------+
// LED BLINKING TASK
//--------------------------------------------------------------------+

void led_blinking_task(__unused void *params) {
  bool on = false;

  // Initialise led
  hard_assert(cyw43_arch_init() == PICO_OK);

  while (1) {
    // Turn led on or off
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
    on = !on;
    vTaskDelay(pdMS_TO_TICKS(LED_DELAY_MS));
  }
}