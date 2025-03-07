#include <Arduino.h>
#include <Wire.h>

// MCP4728 I2C address
#define MCP4728_I2C_ADDRESS 0x60

//  input pin
#define POT_PIN A1

// sine wave parameters
#define MIN_FREQUENCY 10.0      // minimum frequency (Hz)
#define MAX_FREQUENCY 10000.0   // maximum target frequency (Hz)
#define DAC_RESOLUTION 4095     // 12-bit DAC (0-4095)

// HS mode parameters
#define HS_MODE_CLOCK 3400000   // 3.4 MHz for HS mode
#define HS_MASTER_CODE 0x08     // High-Speed Master Mode Code (arbitrary 3 bits: 001)

// timing variables
unsigned long previousMicros = 0;
unsigned long sampleInterval = 0;  // will be calculated based on measured performance
float phase = 0.0;
float currentFrequency = 1000.0;   // starting frequency

// performance measurement
unsigned long updateCount = 0;
unsigned long startTime = 0;
float actualUpdateRate = 0;

// sine lookup table for faster computation
#define TABLE_SIZE 256
uint16_t sineTable[TABLE_SIZE];

// potentiometer reading variables
unsigned long lastPotReadTime = 0;
const unsigned long POT_READ_INTERVAL = 100; // Read pot every 100ms

// status reporting variables
unsigned long lastStatusTime = 0;

// function to directly write to MCP4728 using I2C with HS mode
bool writeMCP4728Channel(uint8_t channel, uint16_t value) {
  // HS Master Mode Code (HSMMC)
  Wire.beginTransmission(0x00);  // Special address for HSMMC
  Wire.write(HS_MASTER_CODE);    // Send HSMMC (0000 1XXX)
  Wire.endTransmission(false);   // No STOP condition yet
  
  // now send normal I2C commands at high speed
  Wire.beginTransmission(MCP4728_I2C_ADDRESS);
  
  // command byte: Multi-write to channel
  // format: 01000 (prefix) + channel (00 to 11)
  Wire.write(0x40 | (channel & 0x03));
  
  // high byte: 0 (VREF bit) + 0 (PD bits) + 0 (G bit) + 4 most significant bits
  Wire.write((value >> 8) & 0x0F);
  
  // low byte: 8 least significant bits
  Wire.write(value & 0xFF);
  
  // end with STOP condition
  return (Wire.endTransmission() == 0);
}

// direct channel definitions for MCP4728
#define CHANNEL_A 0
#define CHANNEL_B 1
#define CHANNEL_C 2
#define CHANNEL_D 3

void setup() {
  // initialize serial communication
  Serial.begin(115200);
  delay(500);
  
  Serial.println("Teensy 4.1 MCP4728 HS-Mode Sine Wave Generator");
  
  // generate sine lookup table for faster computation
  for (int i = 0; i < TABLE_SIZE; i++) {
    float angle = 2.0 * PI * i / TABLE_SIZE;
    sineTable[i] = (uint16_t)((sin(angle) + 1.0) * 2047.5);
  }
  
  // configure potentiometer input
  pinMode(POT_PIN, INPUT);
  analogReadResolution(12); // 12-bit ADC resolution
  
  // initialize I2C
  Wire.begin();
  Wire.setClock(HS_MODE_CLOCK);  // Set to HS mode clock speed
  
  // test communication with MCP4728
  bool success = writeMCP4728Channel(CHANNEL_B, 2048);
  
  if (!success) {
    Serial.println("Failed initial communication with MCP4728");
    Serial.println("Falling back to standard speed");
    
    Wire.setClock(800000);  // fall back to fast mode plus
    
    // test again at standard speed
    success = writeMCP4728Channel(CHANNEL_B, 2048);
    if (!success) {
      Serial.println("Still can't communicate with MCP4728");
      Serial.println("Check connections and try again");
      while (1) delay(100);
    } else {
      Serial.println("MCP4728 responding at standard speed (800 kHz)");
    }
  } else {
    Serial.println("Successfully initialized MCP4728 in HS mode");
  }
  
  // initialize all channels to mid-scale
  writeMCP4728Channel(CHANNEL_A, 2048);
  writeMCP4728Channel(CHANNEL_B, 2048);
  writeMCP4728Channel(CHANNEL_C, 2048);
  writeMCP4728Channel(CHANNEL_D, 2048);
  
  // determine actual update rate capabilities
  Serial.println("\nMeasuring maximum update rate...");
  
  startTime = millis();
  updateCount = 0;
  
  // run test for 1 second to determine actual update rate
  while (millis() - startTime < 1000) {
    writeMCP4728Channel(CHANNEL_B, 2048);
    updateCount++;
  }
  
  actualUpdateRate = (float)updateCount / ((millis() - startTime) / 1000.0);
  
  Serial.print("Maximum DAC update rate: ");
  Serial.print(actualUpdateRate);
  Serial.println(" Hz");
  
  // calculate sample interval based on actual performance
  sampleInterval = 1000000 / actualUpdateRate;
  
  // calculate maximum achievable frequency (assuming minimum 5 samples per cycle)
  float maxFreq = actualUpdateRate / 5.0;
  Serial.print("Maximum recommended frequency for clean sine: ~");
  Serial.print(maxFreq);
  Serial.println(" Hz");
  
  if (MAX_FREQUENCY > maxFreq) {
    Serial.print("Warning: Target max frequency ");
    Serial.print(MAX_FREQUENCY);
    Serial.print(" Hz exceeds recommended ");
    Serial.print(maxFreq);
    Serial.println(" Hz");
    Serial.println("Higher frequencies will have less resolution");
  }
  
  Serial.print("Frequency range: ");
  Serial.print(MIN_FREQUENCY);
  Serial.print(" - ");
  Serial.print(MAX_FREQUENCY);
  Serial.println(" Hz");
  
  startTime = millis();
  updateCount = 0;
}

void loop() {
  // read potentiometer periodically to adjust frequency
  if (millis() - lastPotReadTime >= POT_READ_INTERVAL) {
    lastPotReadTime = millis();
    
    // read potentiometer value (0-4095)
    int potValue = analogRead(POT_PIN);
    
    // map pot value to frequency range logarithmically
    float freqRatio = (float)potValue / 4095.0;  // 0.0 to 1.0
    
    // calculate frequency using log scale
    currentFrequency = MIN_FREQUENCY * pow(MAX_FREQUENCY / MIN_FREQUENCY, freqRatio);
    
    // round to nearest integer for cleaner display
    currentFrequency = round(currentFrequency);
  }
  
  // generate sine wave
  unsigned long currentMicros = micros();
  
  // check if it's time to update the DAC output
  if (currentMicros - previousMicros >= sampleInterval) {
    previousMicros = currentMicros;
    
    // use lookup table for faster sine calculation
    uint8_t index = (uint8_t)(phase * TABLE_SIZE) % TABLE_SIZE;
    uint16_t dacValue = sineTable[index];
    
    // update phase for next sample
    phase += currentFrequency / actualUpdateRate;
    while (phase >= 1.0) {
      phase -= 1.0;  // Keep phase in 0-1 range
    }
    
    // output to DAC channel B
    writeMCP4728Channel(CHANNEL_B, dacValue);
    
    // count updates for statistics
    updateCount++;
  }
  
  // print status every second
  if (millis() - lastStatusTime >= 1000) {
    lastStatusTime = millis();
    
    float measuredUpdateRate = (float)updateCount * 1000.0 / (millis() - startTime);
    
    // calculate samples per cycle at current frequency
    float samplesPerCycle = measuredUpdateRate / currentFrequency;
    
    Serial.print("Freq: ");
    Serial.print(currentFrequency);
    Serial.print(" Hz, Updates: ");
    Serial.print(measuredUpdateRate);
    Serial.print(" Hz, Samples/cycle: ");
    Serial.println(samplesPerCycle);
    
    // reset for next measurement period
    updateCount = 0;
    startTime = millis();
  }
}
