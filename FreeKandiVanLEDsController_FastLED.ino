// includes folder and a lot of the code for processing the audio (fft) comes from here:
// https://github.com/zhujisheng/audio-reactive-led-strip


#include <FastLED.h>
#include <driver/i2s.h>
#include "includes/FFT.h"

// Pin for the wheels (4x 16 LED circles)
#define DATA_PIN 33
// Pin for the LED strip inside the van
#define DATA_PIN_STRIP 35

// Pins for the INMP441 mic
#define PIN_I2S_WS 11//GPIO_NUM_11
#define PIN_IS2_SD 8//GPIO_NUM_8
#define PIN_I2S_SCK 7//GPIO_NUM_7

// ESP32 S2 mini v1.0 LED is GPIO 15
#define BOARD_LED 15

// Button pins
#define BRIGHTNESS_STRIP_PIN 18 // Button pins for the buttons that cycles brightness levels
#define BRIGHTNESS_WHEELS_PIN 37
#define CYCLE_PROGRAM_PIN 21 // Button pin for cycling through the LED programs (for the wheels, might make a separate one for the strip inside the van)
#define CYCLE_AUDIO_SENSITIVITY_PIN 38 // Button 

// Wheels
#define NUM_LEDS 64
#define NUM_LEDS_PER_WHEEL NUM_LEDS / 4
// LED strip inside the van
#define NUM_LEDS_STRIP 64
#define NUM_LEDS_STRIP_HALF NUM_LEDS_STRIP / 2

// Audio processing and more mic stuff
#define BUFFER_SIZE 512
#define SAMPLE_RATE 44100
#define N_ROLLING_HISTORY 1
const uint16_t N_MEL_BIN = 8;
const float MIN_FREQUENCY = 40;
const float MAX_FREQUENCY = 600;
const float MIN_VOLUME_THRESHOLD= 0.01;
float y_data[BUFFER_SIZE * N_ROLLING_HISTORY];
class FFT fft(BUFFER_SIZE*N_ROLLING_HISTORY, N_MEL_BIN, MIN_FREQUENCY, MAX_FREQUENCY, SAMPLE_RATE, MIN_VOLUME_THRESHOLD);
static float mel_data[N_MEL_BIN];
int16_t l[BUFFER_SIZE];
unsigned int read_num;
float minVolumeThreshold[] = {0.0001, 0.001, 0.01, 0.05, 0.1, 0.15, 0.25, 0.35, 0.5};
float ignoreVolumeThreshold[] = {0.02, 0.05, 0.1, 0.17, 0.25, 0.35, 0.45, 0.55, 0.7};
uint8_t currentVolThreshold = 0;


// MIC SETUP
// don't mess around with this
i2s_config_t i2s_config = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
  .sample_rate = SAMPLE_RATE,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
  .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
  .communication_format = I2S_COMM_FORMAT_I2S,
  .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
  .dma_buf_count = 8,
  .dma_buf_len = 64,
  .use_apll = false,
  .tx_desc_auto_clear = false,
  .fixed_mclk = 0
};

// and don't mess around with this
i2s_pin_config_t i2s_mic_pins = {
  .bck_io_num = PIN_I2S_SCK,
  .ws_io_num = PIN_I2S_WS,
  .data_out_num = -1,
  .data_in_num = PIN_IS2_SD
};

// Creates a rainbow palette of 256 colors
CRGBPalette16 currentPalette;
TBlendType currentBlending;

// Arrays for the LED wheels and strip
CRGB leds[NUM_LEDS];
CRGB leds_strip[NUM_LEDS_STRIP];

// Misc global variables for my code
uint8_t currentWheelLedNum = 0;
uint8_t currentRainbowColor = 0;
uint8_t currentRainbowColor_strip = 0;
uint8_t currentBrightnessLevel_strip = 0;
uint8_t currentBrightnessLevel_wheels = 0;
uint8_t brightnessLevels[] = {2, 5, 15, 30, 60, 100};
uint8_t currentProgramSelectionWheels = 0;
uint8_t currentProgramSelectionStrip = 0;
uint8_t pulseSize = 3; // Set this to how big the "pulses" of scrolling LEDs will be - NUM_LEDS_STRIP has to be divisible by this number!
uint16_t stripHalfIterationTracker = NUM_LEDS_STRIP_HALF - pulseSize; // Tracking which is the leading pixel in between function calls

// arrays of functions for visualizers/programs to run
typedef void (* ProgramFunc)();
ProgramFunc StripProgs[4] = {&RainbowPulseScroll_strip, &RainbowScrollFromCenter_strip, &RainbowScroll_strip, &RainbowScroll_strip_AUDIO_REACTIVE};
ProgramFunc WheelProgs[3] = {&SinglePixelRainbow_wheels, &RainbowScroll_wheels, &RainbowCycle_wheels};




// Run once when ESP32 first boots ups
void setup() { 
  pinMode(BOARD_LED, OUTPUT); // LED pin for the wheels
  // digitalWrite(BOARD_LED, HIGH); // Built in esp32 board LED, turn on so I know it's "working"

  pinMode(BRIGHTNESS_STRIP_PIN, INPUT_PULLUP); // Button pin for the brightness control
  pinMode(BRIGHTNESS_WHEELS_PIN, INPUT_PULLUP);
  pinMode(CYCLE_PROGRAM_PIN, INPUT_PULLUP); // Button pin for cycling the LED programs
  pinMode(CYCLE_AUDIO_SENSITIVITY_PIN , INPUT_PULLUP);
  
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);  // GRB ordering is assumed
  FastLED.addLeds<NEOPIXEL, DATA_PIN_STRIP>(leds_strip, NUM_LEDS_STRIP);

  FastLED.setBrightness(255); // Brightness goes from 0-255 - setting it to max here because we're going to control it pixel-by-pixel (really just the 2 separate "strips") in the visualizer functions
  FastLED.setMaxPowerInMilliWatts(2500); // 2500 mW = 2.5 W (500mA at 5v, or 757.57 at 3.3v (when testing on board));

  currentPalette = RainbowColors_p; // Rainbow palette for visuals
  currentBlending = LINEARBLEND; // Even distribution of colors/blending

  // MIC SETUP
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &i2s_mic_pins);

  // Serial.begin(115200);
  digitalWrite(BOARD_LED, HIGH); // Built in esp32 board LED, turn on so I know it's "working"
  // delay(1000);
}


// Runs over and over and over and over
void loop() {
  // digitalWrite(BOARD_LED, LOW);

  // Check if the button for brightness control for the strip is pressed
  if (digitalRead(BRIGHTNESS_STRIP_PIN) == LOW) {
    currentBrightnessLevel_strip += 1;
    if (currentBrightnessLevel_strip > 5) {
      currentBrightnessLevel_strip = 0;
    }
  //  FastLED.setBrightness(brightnessLevels[currentBrightnessLevel_strip]); // Brightness goes from 0-255
    delay(500);
  }

  // Check if the button for brightness control for the wheels is pressed
  if (digitalRead(BRIGHTNESS_WHEELS_PIN) == LOW) {
    currentBrightnessLevel_wheels += 1;
    if (currentBrightnessLevel_wheels > 5) {
      currentBrightnessLevel_wheels = 0;
    }
   // FastLED.setBrightness(brightnessLevels[currentBrightnessLevel_wheels]); // Brightness goes from 0-255
    delay(500);
  }

  // Check if the button to decrease audio sensitivity is pressed (will prob want to turn it down the closer to speakers we are, like turning it down a lot in the pit otherwise it will just be constant LEDs with how loud it is)
  if (digitalRead(CYCLE_AUDIO_SENSITIVITY_PIN) == LOW) {
    currentVolThreshold++;
    if (currentVolThreshold > 8) {
      currentVolThreshold = 0;
    }
    fft._min_volume_threshold = minVolumeThreshold[currentVolThreshold];
  }

  // Check if the button for cycling programs is pressed
  if (digitalRead(CYCLE_PROGRAM_PIN) == LOW) {
    delay(500);
    if (digitalRead(CYCLE_PROGRAM_PIN) == LOW) { // Wait .5 seconds, if still pressed in (long press) then cycle the strip visualizer, else short press cycle the wheels
      currentProgramSelectionStrip++;
      if (currentProgramSelectionStrip >= 3) { // Change this value as the # of visualizers for the strip increases - make a global variable?
        currentProgramSelectionStrip = 0;
      }
    }
    else {
      currentProgramSelectionWheels++;
      if (currentProgramSelectionWheels >= 2) { // Change this value as the # of programs increases
        currentProgramSelectionWheels = 0;
      }
    }
    
    delay(1000); // Wait 1.0 seconds to make sure we don't think the button is pressed down again (give the user time to take finger off button after the change)
  }

  if (currentProgramSelectionStrip > 2 || currentProgramSelectionWheels > 2) {
    // Set up previous reads
    for (int i = 0; i < N_ROLLING_HISTORY - 1; i++)
      memcpy(y_data + i * BUFFER_SIZE, y_data + (i + 1)*BUFFER_SIZE, sizeof(float)*BUFFER_SIZE);

    // Read in audio from mic
    i2s_read(I2S_NUM_0, l, BUFFER_SIZE * 2, &read_num, portMAX_DELAY);

    // Process latest read buffer into the previous reads
    for (int i = 0; i < BUFFER_SIZE; i++) {
      y_data[BUFFER_SIZE * (N_ROLLING_HISTORY - 1) + i] = l[i] / 32768.0;
    }

    // Process the audio
    fft.t2mel( y_data, mel_data );
  }


  EVERY_N_MILLISECONDS(50) { WheelProgs[currentProgramSelectionWheels](); };

  EVERY_N_MILLISECONDS(50) { StripProgs[currentProgramSelectionStrip](); };

  FastLED.show();


  // Debugging via printing the data to serial to see how it looks
  // for (int i = 0; i < N_MEL_BIN; i++)
  // {
  //   Serial.printf("%f\n", mel_data[i]);
  // }



  // // Pixel test using the processed audio
  // RainbowScroll_strip_AUDIO_TEST(mel_data);

  // RainbowScroll_strip();
  // RainbowScroll_wheels();


  // digitalWrite(BOARD_LED, HIGH); // Built in esp32 board LED, turn on so I know it's "working"
  // FastLED.show();
  // FastLED.delay(50);
  
}


void RainbowScroll_strip_AUDIO_REACTIVE() {
  currentRainbowColor_strip = 200;
  mel_data[0] *= 1.4;
  mel_data[1] *= 1.4;
  uint8_t pc = 29;
  for(int i = 0; i < 8; i++) {
    Serial.printf("%f\n", mel_data[i]);
    if (mel_data[i] > ignoreVolumeThreshold[currentVolThreshold]) {
      // if (mel_data[i] > 1.0) {
        // mel_data[i] = 1.0;
      // }
      // uint8_t convertedData = abs(mel_data[i]*200.0);
      // Serial.printf("%c\n", convertedData);
      leds_strip[pc] = ColorFromPalette(currentPalette, currentRainbowColor_strip, mel_data[i]*brightnessLevels[currentBrightnessLevel_strip], currentBlending);
      leds_strip[pc-1] = ColorFromPalette(currentPalette, currentRainbowColor_strip, mel_data[i]*brightnessLevels[currentBrightnessLevel_strip], currentBlending);
      leds_strip[NUM_LEDS_STRIP-pc-1] = ColorFromPalette(currentPalette, currentRainbowColor_strip, mel_data[i]*brightnessLevels[currentBrightnessLevel_strip], currentBlending);
      leds_strip[NUM_LEDS_STRIP-pc] = ColorFromPalette(currentPalette, currentRainbowColor_strip, mel_data[i]*brightnessLevels[currentBrightnessLevel_strip], currentBlending);
      // leds_strip[i].setHSV(currentRainbowColor_strip, 255, 20);
      currentRainbowColor_strip += 28;
    }
    else {
      leds_strip[pc].setHSV(0, 0, 0);
      leds_strip[pc-1].setHSV(0, 0, 0);
      leds_strip[NUM_LEDS_STRIP-pc-1].setHSV(0, 0, 0);
      leds_strip[NUM_LEDS_STRIP-pc].setHSV(0, 0, 0);
    }
    pc -= 4;
  }
  // currentRainbowColor_strip = currentRainbowColor_strip - NUM_LEDS_STRIP + 1;
}



void RainbowPulseScroll_strip() {
  // for(int i = stripHalfIterationTracker; i >= stripHalfIterationTracker - pulseSize; i -+ pulseSize) {
  for (int i = 0; i < pulseSize; i++) {
    if (stripHalfIterationTracker + i >= 0 && stripHalfIterationTracker + i < NUM_LEDS_STRIP_HALF) {
      leds_strip[stripHalfIterationTracker + i] = ColorFromPalette(currentPalette, currentRainbowColor_strip, brightnessLevels[currentBrightnessLevel_strip], currentBlending);
      // leds_strip[NUM_LEDS_STRIP - NUM_LEDS_STRIP_HALF + stripHalfIterationTracker - j] = ColorFromPalette(currentPalette, currentRainbowColor_strip, 255, currentBlending);
    }
  }
  // Clear the last pixel
  if (stripHalfIterationTracker + pulseSize + pulseSize > NUM_LEDS_STRIP_HALF) {
    leds_strip[stripHalfIterationTracker + pulseSize + pulseSize - NUM_LEDS_STRIP_HALF - 1].setHSV(0, 0, 0);
  }
  else {
    leds_strip[stripHalfIterationTracker + pulseSize].setHSV(0, 0, 0);
  }
  if (stripHalfIterationTracker == 0) {
    stripHalfIterationTracker = NUM_LEDS_STRIP_HALF - pulseSize;
  }
  else {
    stripHalfIterationTracker--;
  }
  currentRainbowColor_strip++;
}


void RainbowScroll_strip() {
  for(int i = 0; i < NUM_LEDS_STRIP; i++) {
    leds_strip[NUM_LEDS_STRIP - i - 1] = ColorFromPalette(currentPalette, currentRainbowColor_strip, brightnessLevels[currentBrightnessLevel_strip], currentBlending);
    // leds_strip[i].setHSV(currentRainbowColor_strip, 255, 20);
    currentRainbowColor_strip++;
  }
  currentRainbowColor_strip = currentRainbowColor_strip - NUM_LEDS_STRIP + 1;
}


void RainbowScrollFromCenter_strip() {
  for(int i = 0; i < NUM_LEDS_STRIP_HALF; i++) {
    leds_strip[NUM_LEDS_STRIP_HALF - i] = ColorFromPalette(currentPalette, currentRainbowColor_strip, brightnessLevels[currentBrightnessLevel_strip], currentBlending);
    leds_strip[i + NUM_LEDS_STRIP_HALF] = ColorFromPalette(currentPalette, currentRainbowColor_strip, brightnessLevels[currentBrightnessLevel_strip], currentBlending);
    // leds_strip[i].setHSV(currentRainbowColor_strip, 255, 20);
    currentRainbowColor_strip++;
  }
  currentRainbowColor_strip = currentRainbowColor_strip - NUM_LEDS_STRIP_HALF + 1;
}



// All LEDs same color and change together, cycling through colors of the rainbow
void RainbowCycle_wheels() {
  for(int i = 0; i < NUM_LEDS; i++) {
    leds[i] = ColorFromPalette(currentPalette, currentRainbowColor, brightnessLevels[currentBrightnessLevel_wheels], currentBlending);
    // leds[i+16] = ColorFromPalette(currentPalette, currentRainbowColor, 255, currentBlending);
    // leds[i+32] = ColorFromPalette(currentPalette, currentRainbowColor, 255, currentBlending);
    // leds[i+48] = ColorFromPalette(currentPalette, currentRainbowColor, 255, currentBlending);
  }
  currentRainbowColor += 2;
  // FastLED.show();
  // FastLED.delay(10);
}


// Wheels will spin with only 1 pixel on, cycling through colors of the rainbow
void SinglePixelRainbow_wheels() {
  if (currentWheelLedNum > 15) {
    currentWheelLedNum = 0;
  }
  // FastLED.clear();
  for (int i = 0; i < NUM_LEDS_PER_WHEEL; i++) {
    if (i == currentWheelLedNum) {
      leds[i] = ColorFromPalette(currentPalette, currentRainbowColor, brightnessLevels[currentBrightnessLevel_wheels], currentBlending);
      leds[i+16] = ColorFromPalette(currentPalette, currentRainbowColor, brightnessLevels[currentBrightnessLevel_wheels], currentBlending);
      leds[i+32] = ColorFromPalette(currentPalette, currentRainbowColor, brightnessLevels[currentBrightnessLevel_wheels], currentBlending);
      leds[i+48] = ColorFromPalette(currentPalette, currentRainbowColor, brightnessLevels[currentBrightnessLevel_wheels], currentBlending);
    }
    else {
        leds[i].setHSV(0, 0, 0);
        leds[i+16].setHSV(0, 0, 0);
        leds[i+32].setHSV(0, 0, 0);
        leds[i+48].setHSV(0, 0, 0);
    }
  }

  currentRainbowColor++;
  currentWheelLedNum++;
  // FastLED.show();
  // FastLED.delay(10);
}


// Wheels will spin in rainbow colors
// From NeverPlayLegit on github: https://github.com/NeverPlayLegit/Rainbow-Fader-FastLED/tree/master
void RainbowScroll_wheels() {
  for(int i = 0; i < NUM_LEDS_PER_WHEEL; i++) {
    leds[i] = ColorFromPalette(currentPalette, currentRainbowColor, brightnessLevels[currentBrightnessLevel_wheels], currentBlending);
    leds[i+16] = ColorFromPalette(currentPalette, currentRainbowColor, brightnessLevels[currentBrightnessLevel_wheels], currentBlending);
    leds[i+32] = ColorFromPalette(currentPalette, currentRainbowColor, brightnessLevels[currentBrightnessLevel_wheels], currentBlending);
    leds[i+48] = ColorFromPalette(currentPalette, currentRainbowColor, brightnessLevels[currentBrightnessLevel_wheels], currentBlending);
    currentRainbowColor++;
  }
  currentRainbowColor = currentRainbowColor - NUM_LEDS_PER_WHEEL + 5;
  // FastLED.show();
  // FastLED.delay(10);
}

