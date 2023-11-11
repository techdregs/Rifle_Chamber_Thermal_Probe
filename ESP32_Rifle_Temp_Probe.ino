// Include necessary libraries for the TFT display
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <driver/adc.h>
#include "esp_adc_cal.h"
#include <SPI.h>

// Define pins for the thermistors and battery voltage measurement
const int AmbientPin = 2;
const int ThroatPin = 12;
const int ChamberPin = 13;
const int ExternalPin = 15;
const int BatteryPin = 34;

// Constants related to thermistor calculations
const float R0 = 100000.0;       // Nominal resistance of the thermistor at 25°C
const float T0_thermistor = 298.15; // Temperature at 25°C in Kelvin
const float B = 3950.0;         // Beta coefficient
const float series_resistor = 99600.0; //Actual measured value of the series resistors in the dividers

const int numSamples = 10;      // Number of samples for averaging
float samplesAmbient[numSamples];  // Arrays to store samples for each thermistor
float samplesThroat[numSamples];
float samplesChamber[numSamples];
float samplesExternal[numSamples];
float samplesBattery[numSamples];  // Array to store battery voltage samples
int sampleIndexAmbient = 0; // Indexes to keep track of current samples
int sampleIndexThroat = 0;
int sampleIndexChamber = 0;
int sampleIndexExternal = 0;
int sampleIndexBattery = 0;

// Variables to store previous values for comparison to handle screen refresh
float prevAmbientTemp = 0;
float prevThroatTemp = 0;
float prevChamberTemp = 0;
float prevExternalTemp = 0;
float prevBatteryVoltage = 0;

// Timing constants and variables
unsigned long lastSampleTime = 0;       // Timestamp of the last sensor sampling
const int sampleInterval = 10;          // Sample interval in ms (100 times per second)
unsigned long lastDisplayUpdateTime = 0; // Timestamp of the last screen update
const int displayUpdateInterval = 100;  // Display update interval in ms (10 times per second)

// Setup for the ST7789 TFT display using Software SPI
#define TFT_MOSI 19    // MOSI pin
#define TFT_SCLK 18    // Serial clock pin
#define TFT_CS   5     // Chip select pin
#define TFT_RST  23    // Reset pin
#define TFT_DC   16    // Data/command pin
#define TFT_BL   4     // Backlight control pin
SPIClass hspi(HSPI);
Adafruit_ST7789 tft = Adafruit_ST7789(&hspi, TFT_CS, TFT_DC, TFT_RST);

//ADC calibration variable
esp_adc_cal_characteristics_t adc_chars;

void setup() {
  Serial.begin(115200);  // Initialize the serial port for debugging (optional)
  hspi.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);

  // ADC calibration
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize((adc_unit_t)ADC_UNIT_1, (adc_atten_t)ADC_ATTEN_DB_11, (adc_bits_width_t)ADC_WIDTH_BIT_12, 1100, &adc_chars);

  // Initialize the TFT display with the correct orientation
  tft.init(135, 240);
  tft.setRotation(1);
  tft.setTextSize(2);  // Set a smaller text size for better fitting on the screen
  tft.fillScreen(ST77XX_BLACK);  // Set background color to black
  tft.setTextColor(ST77XX_WHITE);  // Set text color to white

  pinMode(TFT_BL, OUTPUT);  // Set the backlight pin as output
  digitalWrite(TFT_BL, HIGH); // Turn on the backlight
}

void loop() {
  unsigned long currentMillis = millis();  // Get the current time in ms
  
  // Sample sensors and update running averages at the specified interval
  if (currentMillis - lastSampleTime >= sampleInterval) {
    lastSampleTime = currentMillis;
    updateAverage(AmbientPin, samplesAmbient, sampleIndexAmbient);
    updateAverage(ThroatPin, samplesThroat, sampleIndexThroat);
    updateAverage(ChamberPin, samplesChamber, sampleIndexChamber);
    updateAverage(ExternalPin, samplesExternal, sampleIndexExternal);
    updateAverage(BatteryPin, samplesBattery, sampleIndexBattery);
  }
  
  // Update the TFT display at the specified interval
  if (currentMillis - lastDisplayUpdateTime >= displayUpdateInterval) {
    lastDisplayUpdateTime = currentMillis;

    // Display thermistor readings or "NCON" if the sensor is not connected
    updateTemperatureDisplay("PA: ", samplesAmbient, 5, 5, prevAmbientTemp);
    updateTemperatureDisplay("PT: ", samplesThroat, 5, 35, prevThroatTemp);
    updateTemperatureDisplay("PC: ", samplesChamber, 5, 65, prevChamberTemp);
    updateTemperatureDisplay("PE: ", samplesExternal, 5, 95, prevExternalTemp);

    // Display battery voltage
    tft.setTextSize(1);
    tft.setCursor(5, 125);
    float currentBatteryVoltage = getBatteryVoltage(samplesBattery);
    if (abs(currentBatteryVoltage - prevBatteryVoltage) > 0.01) {
      tft.fillRect(5, 125, 135, 10, ST77XX_BLACK); // Clear previous reading
      tft.print("Batt: "); tft.print(currentBatteryVoltage, 2); tft.println(" V");
      prevBatteryVoltage = currentBatteryVoltage;
    }
    tft.setTextSize(2);
  }
}

// Function to update the running average for thermistor/battery readings
void updateAverage(int pin, float samples[], int &currentSampleIndex) {
  uint16_t rawReading = analogRead(pin);
  
  // Convert raw ADC reading to calibrated voltage
  float measuredVoltage = esp_adc_cal_raw_to_voltage(rawReading, &adc_chars) / 1000.0;  // Convert mV to V
  
  // If not the battery pin, compute resistance of thermistor and store
  if (pin != BatteryPin) {
    float V_thermistor = measuredVoltage;  // Directly use the measured voltage as voltage across the thermistor
    float R_fixed = series_resistor;  // Actual resistance value of the fixed resistor in ohms
    float resistance = (V_thermistor * R_fixed) / (3.3 - V_thermistor);  // Corrected resistance calculation

    samples[currentSampleIndex] = resistance;  // Store the resistance value in the sample array
  } else {
    // For battery
    samples[currentSampleIndex] = measuredVoltage * 2;  // Store the doubled voltage value in the sample array for battery
  }
  
  currentSampleIndex = (currentSampleIndex + 1) % numSamples;  // Update the current sample index
}


// Function to calculate temperature in Fahrenheit from resistance values
float getTemperature(float samples[]) {
  float avgResistance = 0;
  for (int i = 0; i < numSamples; i++) {
    avgResistance += samples[i];
  }
  avgResistance /= numSamples;  // Calculate the average resistance

  // Calculate temperature in Kelvin using the Beta coefficient equation
  float tempK = 1.0 / (log(avgResistance / R0) / B + (1.0 / T0_thermistor));
  float tempC = tempK - 273.15;  // Convert Kelvin to Celsius
  float tempF = tempC * 9.0 / 5.0 + 32;  // Convert Celsius to Fahrenheit
  
  return tempF;  // Return temperature in Fahrenheit
}

// Function to get the average battery voltage from samples
float getBatteryVoltage(float samples[]) {
  float avgVoltage = 0;
  for (int i = 0; i < numSamples; i++) {
    avgVoltage += samples[i];
  }
  return avgVoltage / numSamples;  // Return the average voltage
}

// Function to update temperature display
void updateTemperatureDisplay(const char* label, float samples[], int x, int y, float &prevTemp) {
  float currentTemp = getTemperature(samples);

  // Check for significant change in temperature or disconnected probe
  if (abs(currentTemp - prevTemp) > 0.1 || currentTemp < -10) {  // Check if the value has changed or is "NCON"
    tft.fillRect(x, y, 135, 20, ST77XX_BLACK); // Clear previous reading area
    tft.setCursor(x, y);
    tft.print(label);
    if (currentTemp < -10) {  // Arbitrary threshold for "NCON", should work for both F or C
      tft.print("NCON");
    } else {
      tft.print(currentTemp, 1); tft.println(" F");
    }
    prevTemp = currentTemp;
  }
}
