#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4728.h>
#include <ADC.h>
#include <ADC_util.h>

// MCP4728 I2C address
#define MCP4728_I2C_ADDRESS 0x60

// sine wave parameters - carrier and modulator
#define CARRIER_FREQUENCY 20000.0     // 1 kHz carrier wave
#define MODULATOR_FREQUENCY 4000.0    // 100 Hz modulator wave
#define SAMPLE_RATE 400000            // 40kHz sample rate
#define DAC_RESOLUTION 4095          // 12-bit DAC (0-4095)

// modulation depth (0.0-1.0, where 1.0 is 100% modulation)
#define MODULATION_DEPTH 0.8

// analog input pin to read the DAC output
#define ANALOG_READ_PIN A0

// create MCP4728 DAC object
Adafruit_MCP4728 dac;

// ADC object
ADC *adc = new ADC();

// precompute wave table
#define WAVE_TABLE_SIZE 1000
uint16_t carrierTable[WAVE_TABLE_SIZE];
uint16_t modulatorTable[WAVE_TABLE_SIZE];

// buffer for visualization
#define BUFFER_SIZE 200
uint16_t adcBuffer[BUFFER_SIZE];
uint16_t bufferIndex = 0;
bool bufferFull = false;

// debug variables
unsigned long lastPrintTime = 0;
unsigned long sampleCounter = 0;

void setup() {
  // initialize serial communication
  Serial.begin(115200); // more reliable baud rate
  delay(500);
  
  Serial.println("Starting AM modulation");
  
  // initialize I2C with high-speed setting
  Wire.begin();
  Wire.setClock(3400000); // 3.4 MHz I2C
  Serial.println("I2C initialized at 3.4 MHz");
  
  // initialize MCP4728 DAC
  if (!dac.begin(MCP4728_I2C_ADDRESS)) {
    Serial.println("Failed to find MCP4728 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("DAC initialized");
  
  // initialize all channels to mid-scale
  dac.setChannelValue(MCP4728_CHANNEL_A, 2048);
  dac.setChannelValue(MCP4728_CHANNEL_B, 2048);
  dac.setChannelValue(MCP4728_CHANNEL_C, 2048);
  dac.setChannelValue(MCP4728_CHANNEL_D, 2048);
  
  // configure ADC for good balance of speed and reliability
  adc->adc0->setAveraging(0);
  adc->adc0->setResolution(12);
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);
  adc->adc0->setReference(ADC_REFERENCE::REF_3V3);
  Serial.println("ADC configured");
  
  // generate sine tables
  for (int i = 0; i < WAVE_TABLE_SIZE; i++) {
    float angle = 2.0 * PI * i / WAVE_TABLE_SIZE;
    
    // carrier is normal sine wave (range -1 to 1)
    float carrierValue = sin(angle);
    carrierTable[i] = (uint16_t)((carrierValue + 1.0) * DAC_RESOLUTION / 2.0);
    
    // modulator is also sine wave but shifted to 0 to 1 range
    // and scaled according to modulation depth
    float modulatorValue = (1.0 - MODULATION_DEPTH) + MODULATION_DEPTH * (0.5 + 0.5 * sin(angle));
    modulatorTable[i] = (uint16_t)(modulatorValue * 1000); // Store scaled by 1000
  }
  
  Serial.println("Sine tables generated");
  Serial.print("Carrier: ");
  Serial.print(CARRIER_FREQUENCY);
  Serial.print("Hz, Modulator: ");
  Serial.print(MODULATOR_FREQUENCY);
  Serial.println("Hz");
  Serial.println("Starting output...");
  
  // let serial output finish
  delay(100);
}

void loop() {
  static unsigned long previousMicros = 0;
  static uint16_t carrierIndex = 0;
  static uint16_t modulatorIndex = 0;
  
  unsigned long currentMicros = micros();
  unsigned long sampleInterval = 1000000 / SAMPLE_RATE;
  
  // time to update DAC and ADC?
  if (currentMicros - previousMicros >= sampleInterval) {
    previousMicros = currentMicros;
    sampleCounter++;
    
    // get sine values from tables
    uint16_t carrierValue = carrierTable[carrierIndex];
    uint16_t modulatorScaled = modulatorTable[modulatorIndex];
    
    // apply amplitude modulation
    float normalizedCarrier = (float)(carrierValue) / DAC_RESOLUTION;
    float normalizedModulator = (float)(modulatorScaled) / 1000.0;
    uint16_t modulatedValue = (uint16_t)(normalizedCarrier * normalizedModulator * DAC_RESOLUTION);
    
    // output to DAC
    dac.setChannelValue(MCP4728_CHANNEL_B, modulatedValue);
    
    // update indices for next sample
    uint16_t carrierIncrement = (uint16_t)(WAVE_TABLE_SIZE * CARRIER_FREQUENCY / SAMPLE_RATE);
    uint16_t modulatorIncrement = (uint16_t)(WAVE_TABLE_SIZE * MODULATOR_FREQUENCY / SAMPLE_RATE);
    
    carrierIndex = (carrierIndex + carrierIncrement) % WAVE_TABLE_SIZE;
    modulatorIndex = (modulatorIndex + modulatorIncrement) % WAVE_TABLE_SIZE;
    
    // read ADC and store in buffer
    if (!bufferFull) {
      adcBuffer[bufferIndex] = adc->adc0->analogRead(ANALOG_READ_PIN);
      bufferIndex++;
      
      if (bufferIndex >= BUFFER_SIZE) {
        bufferFull = true;
      }
    }
  }
  
  // print status every second
  unsigned long currentMillis = millis();
  if (currentMillis - lastPrintTime >= 1000) {
    lastPrintTime = currentMillis;
    Serial.print("Running... Samples: ");
    Serial.println(sampleCounter);
  }
  
  // if buffer is full => print values
  if (bufferFull) {
    // print data header
    Serial.print("# Samples: ");
    Serial.println(sampleCounter);
    
    // print all buffer values on separate lines for Serial Plotter
    for (int i = 0; i < BUFFER_SIZE; i++) {
      Serial.println(adcBuffer[i]);
    }
    
    // reset buffer
    bufferFull = false;
    bufferIndex = 0;
    
    // delay to let serial transmission complete
    delay(50);
  }
}
