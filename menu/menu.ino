#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "AiEsp32RotaryEncoder.h"
#include "Arduino.h"

// Pin Definitions
#define ROTARY_ENCODER_A_PIN 23
#define ROTARY_ENCODER_B_PIN 18
#define ROTARY_ENCODER_BUTTON_PIN 19
#define ROTARY_ENCODER_VCC_PIN -1
#define ROTARY_ENCODER_STEPS 4

// ESP32 Pin Assignments
const int HEATER_PWM_PIN = 27;
const int THERMISTOR_PIN = 33;

// PWM Power Levels
const int MAX_POWER = 255;
const int LOW_POWER = 10;
const int UPKEEP_POWER = 39;
const int TEMPERING_POWER = 9;

// Timing Constants (seconds)
const int MAX_POWER_DURATION = 130;
const int LOW_POWER_DURATION = 60;

// Global Variables
int temperingPhase = 0;

// Initialize LCD (I2C address, columns, rows)
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Initialize Rotary Encoder
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(
  ROTARY_ENCODER_A_PIN, 
  ROTARY_ENCODER_B_PIN, 
  ROTARY_ENCODER_BUTTON_PIN, 
  ROTARY_ENCODER_VCC_PIN, 
  ROTARY_ENCODER_STEPS
);

// ISR for Rotary Encoder
void IRAM_ATTR readEncoderISR() {
  rotaryEncoder.readEncoder_ISR();
}

void setup() {
  Serial.begin(115200);
  
  initLCD();
  initEncoder();
  
  pinMode(HEATER_PWM_PIN, OUTPUT);  // Heating resistor driver
  pinMode(THERMISTOR_PIN, INPUT);   // Thermistor input after voltage divider
  pinMode(LED_BUILTIN, OUTPUT);     // Status LED output
}

void loop() {
  // Read temperature from thermistor
  int temperature = readThermistor(THERMISTOR_PIN);
  
  // Calculate appropriate power level based on current phase and timing
  int pwmPower = calculateHeaterPower();
  
  // Update LCD display
  updateLCD(pwmPower, temperature);
  
  // Set heater power
  analogWrite(HEATER_PWM_PIN, pwmPower);
  
  // Handle rotary encoder input
  handleEncoder();
  
  delay(10);
}

// Initialize LCD
void initLCD() {
  lcd.init();
  lcd.backlight();
}

// Initialize Rotary Encoder
void initEncoder() {
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  rotaryEncoder.setBoundaries(0, 7, true);  // Min, max values, circular (wrap)
  rotaryEncoder.setAcceleration(250);       // Higher value = more acceleration
}

// Update all LCD fields
void updateLCD(int pwmPower, int temperature) {
  lcd.setCursor(0, 0);
  lcd.print("Mode:" + String(temperingPhase) + "  ");

  lcd.setCursor(0, 1);
  lcd.print("PWM: " + String(pwmPower) + "   ");

  lcd.setCursor(0, 2);
  lcd.print("Resistance: " + String(temperature));

  lcd.setCursor(0, 3);
  lcd.print("Time: " + String(millis()/1000) + "s");
}

// Calculate heater power based on current phase and timing
int calculateHeaterPower() {
  unsigned long currentTimeMs = millis();
  unsigned long currentTimeSec = currentTimeMs / 1000;
  
  // If in tempering phase (set by button press)
  if (temperingPhase == 1) {
    return TEMPERING_POWER;
  }
  
  // Convert timing constants to milliseconds
  unsigned long maxPowerEndTimeMs = MAX_POWER_DURATION * 1000;
  unsigned long lowPowerEndTimeMs = maxPowerEndTimeMs + (LOW_POWER_DURATION * 1000);
  
  // Initial heating phase
  if (currentTimeMs < maxPowerEndTimeMs) {
    return MAX_POWER;
  }
  
  // Cooling phase
  if (currentTimeMs < lowPowerEndTimeMs) {
    return LOW_POWER;
  }
  
  // Maintenance phase
  return UPKEEP_POWER;
}

// Read and average thermistor values
int readThermistor(int pin) {
  const int SAMPLE_COUNT = 50;
  int readings[SAMPLE_COUNT];
  
  // Take multiple readings
  for (int i = 0; i < SAMPLE_COUNT-1; i++) {
    readings[i] = analogRead(pin);
    delay(1);
  }
  
  // Calculate sum of readings
  long sum = 0;
  for (int i = 0; i < SAMPLE_COUNT-1; i++) {
    sum += readings[i];
  }
  
  // Return average (note: the original calculation divided by 1000 instead of SAMPLE_COUNT)
  // Keeping this as-is to maintain exact functionality
  return (int)(sum / 1000);
}

// Handle rotary encoder events
void handleEncoder() {
  // Check for encoder value changes
  if (rotaryEncoder.encoderChanged()) {
    Serial.println("trash\n");
    // Original code had this empty functionality, keeping it for exact behavior
  }
  
  // Check for button press using direct GPIO read
  if (digitalRead(ROTARY_ENCODER_BUTTON_PIN) == LOW) {
    Serial.println("Direct GPIO Detect! Button is LOW");
    onEncoderButtonPressed();
    delay(50);  // Debounce
  }
}

// Handle encoder button press
void onEncoderButtonPressed() {
  Serial.print("button pressed ");
  temperingPhase = 1;
}



