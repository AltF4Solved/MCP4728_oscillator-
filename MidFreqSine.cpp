#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4728.h>

// MCP4728 I2C address
#define MCP4728_I2C_ADDRESS 0x60

// potentiometer input pin
#define POT_PIN A1  // Change this to match your potentiometer connection

// sine wave parameters
#define MIN_FREQUENCY 10.0      // minimum frequency (Hz)
#define MAX_FREQUENCY 5000.0    // maximum frequency (Hz)
#define SAMPLE_RATE 10000       // samples per second for DAC updates
#define DAC_RESOLUTION 4095     // 12-bit DAC (0-4095)

// MCP4728 DAC object
Adafruit_MCP4728 dac;

// timing variables
unsigned long previousMicros = 0;
const unsigned long sampleInterval = 1000000 / SAMPLE_RATE; // in microseconds
float phase = 0.0;
float currentFrequency = 1000.0; // starting frequency

// potentiometer/pitch-in reading variables
unsigned long lastPotReadTime = 0;
const unsigned long POT_READ_INTERVAL = 50; // read pin every 50ms

// status reporting variables
unsigned long lastStatusTime = 0;
unsigned long updateCount = 0;
unsigned long startTime = 0;

void setup() {
  // initialize serial communication, remove if not debugging
  Serial.begin(115200);
  delay(500);
  
  Serial.println("Variable Frequency Sine Wave Generator using the MCP4728");
  
  // initialize I2C
  Wire.begin();
  Wire.setClock(800000); // set I2C speed to 800 kHz (Fast Mode Plus), can reduce to 400 kHz 
  
  // initialize MCP4728 DAC
  if (!dac.begin(MCP4728_I2C_ADDRESS)) {
    Serial.println("Cant find MCP4728 chip");
    while (1) {
      delay(10);
    }
  }
  
  Serial.println("MCP4728 DAC initialized successfully");
  
  // configure potentiometer input
  pinMode(POT_PIN, INPUT);
  analogReadResolution(12); // 12-bit ADC resolution for pot
  
  // initialize all channels to mid-scale
  dac.setChannelValue(MCP4728_CHANNEL_A, 2048);
  dac.setChannelValue(MCP4728_CHANNEL_B, 2048);
  dac.setChannelValue(MCP4728_CHANNEL_C, 2048);
  dac.setChannelValue(MCP4728_CHANNEL_D, 2048);
  
  Serial.print("Frequency range: ");
  Serial.print(MIN_FREQUENCY);
  Serial.print(" - ");
  Serial.print(MAX_FREQUENCY);
  Serial.println(" Hz");
  
  Serial.print("Starting frequency: ");
  Serial.print(currentFrequency);
  Serial.println(" Hz");
  
  Serial.print("Sample rate: ");
  Serial.print(SAMPLE_RATE);
  Serial.println(" Hz");
  
  Serial.print("Turn potentiometer on pin A");
  Serial.print(POT_PIN - A0);  // convert to display just the analog pin number
  Serial.println(" to adjust frequency");
  
  startTime = millis();
}

void loop() {
  // read input pin periodically to adjust frequency
  if (millis() - lastPotReadTime >= POT_READ_INTERVAL) {
    lastPotReadTime = millis();
    
    // read pin value (0-4095)
    int potValue = analogRead(POT_PIN);
    
    // map pot value to frequency range (use logarithmic mapping for natural hearing)
    // log scale also gives better control over wide frequency range
    float freqRatio = (float)potValue / DAC_RESOLUTION;  // 0.0 to 1.0
    
    // logarithmic mapping: freq = min_freq * (max_freq/min_freq)^pot_ratio
    currentFrequency = MIN_FREQUENCY * pow(MAX_FREQUENCY / MIN_FREQUENCY, freqRatio);
    
    //  round to nearest integer for cleaner display
    currentFrequency = round(currentFrequency);
  }
  
  // generate sine wave
  unsigned long currentMicros = micros();
  
  // check if it's time to update the DAC output
  if (currentMicros - previousMicros >= sampleInterval) {
    previousMicros = currentMicros;
    
    // calculate sine value (-1 to 1)
    float sinValue = sin(2 * PI * currentFrequency * phase);
    
    // scale to DAC range (0-4095)
    int dacValue = (int)((sinValue + 1.0) * DAC_RESOLUTION / 2.0);
    
    // update phase for next sample
    phase += 1.0 / SAMPLE_RATE;
    if (phase >= 1.0 / currentFrequency) {
      phase = 0.0;
    }
    
    // output to DAC channel B
    dac.setChannelValue(MCP4728_CHANNEL_B, dacValue);
    
    // count updates for statistics
    updateCount++;
  }
  
  // print status every second
  if (millis() - lastStatusTime >= 1000) {
    lastStatusTime = millis();
    unsigned long elapsedTime = millis() - startTime;
    float actualUpdateRate = (float)updateCount * 1000.0 / elapsedTime;
    
    Serial.print("Current frequency: ");
    Serial.print(currentFrequency);
    Serial.print(" Hz, Updates: ");
    Serial.print(actualUpdateRate);
    Serial.println(" Hz");
    
    // reset counter for next second
    updateCount = 0;
    startTime = millis();
  }
}
