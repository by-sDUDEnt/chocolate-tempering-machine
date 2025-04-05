#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "AiEsp32RotaryEncoder.h"
#include "Arduino.h"

// Pin definitions
#define ROTARY_ENCODER_A_PIN 23
#define ROTARY_ENCODER_B_PIN 18
#define ROTARY_ENCODER_BUTTON_PIN 19
#define ROTARY_ENCODER_VCC_PIN -1
#define ROTARY_ENCODER_STEPS 4

// Output pins
const int DRIVER_PWM_PIN = 27;  // Heating resistor driver
const int THERMISTOR_PIN = 33;  // Thermistor input after voltage divider

// Thermistor parameters
#define THERMISTOR_NOMINAL 10000    // 10k ohm at 25°C
#define TEMPERATURE_NOMINAL 25      // 25°C
#define BETA_COEFFICIENT 3950       // Beta coefficient of the thermistor
#define SERIES_RESISTOR 10000       // 10k ohm series resistor
#define ADC_MAX 4095                // 12-bit ADC max value
#define NUM_SAMPLES 50              // Number of samples for temperature reading

// Safety parameters
#define MAX_SAFE_TEMP 50            // Maximum safe temperature in °C
#define MIN_VALID_TEMP 0            // Minimum valid temperature in °C
#define MAX_VALID_TEMP 100          // Maximum valid temperature in °C

// Menu system
const char* MENU_TITLES[4] = {"Mode: ", "PWM: ", "Temp: ", "Time: "};
const char* MODES[2] = {"manual", "auto"};

// Device settings
uint8_t temperator_mode = 0;       // 0 = manual, 1 = auto
uint8_t temperator_pwm = 0;        // PWM value (0-255)
unsigned long start_time = 0;       // Start time for auto mode
bool heating_enabled = true;        // Safety flag to disable heating

// Menu state
uint8_t menuItemIndex = 0;
bool isMenuItemPicked = false;
uint8_t onScreenPWM = 0;
float current_temp = 0.0;
unsigned long elapsed_time = 0;
char time_buffer[10];              // Buffer for time display

// Auto mode parameters
typedef struct {
    float target_temp;
    uint8_t pwm_value;
    unsigned long duration;
} TempPhase;

// Define tempering phases
const TempPhase TEMP_PHASES[] = {
    {45.0, 255, 130000},  // Phase 1: Heat to 45°C
    {27.0, 10,  60000},   // Phase 2: Cool to 27°C
    {31.5, 39,  0}        // Phase 3: Maintain at 31.5°C
};
uint8_t current_phase = 0;

// Hardware initialization
LiquidCrystal_I2C lcd(0x27, 20, 4);  // LCD with I2C address 0x27, 20 columns, 4 rows
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(
  ROTARY_ENCODER_A_PIN, 
  ROTARY_ENCODER_B_PIN, 
  ROTARY_ENCODER_BUTTON_PIN, 
  ROTARY_ENCODER_VCC_PIN, 
  ROTARY_ENCODER_STEPS
);

// Function prototypes
void lcd_setup();
void encoder_setup();
float readTemperature();
void handleTemperator();
void changeTemperatorPWM(uint8_t pwm);
uint8_t getAutoCurrentPower();
void lcd4rowUpdate();
String padString(const String& text);
void rotary_loop();
void handle_spin();
void change_mode();
void change_pwm();
void change_time();
void rotary_onButtonClick();
bool checkSafety();

// Interrupt Service Routine for rotary encoder
void IRAM_ATTR readEncoderISR() {
  rotaryEncoder.readEncoder_ISR();
}

void setup() {
  Serial.begin(115200);
  
  // Initialize hardware
  lcd_setup();
  encoder_setup();
  
  // Configure pins
  pinMode(DRIVER_PWM_PIN, OUTPUT);  // Heating resistor driver
  pinMode(THERMISTOR_PIN, INPUT);   // Thermistor input
  pinMode(LED_BUILTIN, OUTPUT);     // Status LED
  
  // Initialize system
  start_time = millis();
  analogWrite(DRIVER_PWM_PIN, 0);   // Start with heating off
}

void loop() {
  // Read temperature and update timing
  current_temp = readTemperature();
  elapsed_time = millis() - start_time;
  
  // Check system safety
  heating_enabled = checkSafety();
  
  // Update LCD display
  lcd4rowUpdate();
  
  // Control temperature
  handleTemperator();
  
  // Check for user input from rotary encoder
  rotary_loop();
  
  // Short delay to avoid busy-waiting
  delay(10);
}

// Setup functions
void lcd_setup() {
  lcd.init();
  lcd.backlight();
}

void encoder_setup() {
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  rotaryEncoder.setBoundaries(0, 3, true);  // minValue, maxValue, cycle (when max goes to min and vice versa)
  rotaryEncoder.setAcceleration(0);  // 0 means disabled acceleration
}

// Safety check function
bool checkSafety() {
  // Check for sensor errors (unreasonable values)
  if (current_temp < MIN_VALID_TEMP || current_temp > MAX_VALID_TEMP) {
    digitalWrite(LED_BUILTIN, HIGH); // Turn on warning LED
    return false;
  }
  
  // Check for overheating
  if (current_temp > MAX_SAFE_TEMP) {
    digitalWrite(LED_BUILTIN, HIGH); // Turn on warning LED
    return false;
  }
  
  digitalWrite(LED_BUILTIN, LOW); // Turn off warning LED
  return true;
}

// Temperature control functions
void handleTemperator() {
  if (MODES[temperator_mode] == "manual") {
    changeTemperatorPWM(onScreenPWM);  // Use the onScreenPWM value in manual mode
  } else if (MODES[temperator_mode] == "auto") {
    changeTemperatorPWM(getAutoCurrentPower());
  }

  // Apply the PWM value to the driver (with safety check)
  if (heating_enabled) {
    analogWrite(DRIVER_PWM_PIN, temperator_pwm);
  } else {
    analogWrite(DRIVER_PWM_PIN, 0);  // Safety cut-off
  }
}

void changeTemperatorPWM(uint8_t pwm) {
  temperator_pwm = pwm;
}

uint8_t getAutoCurrentPower() {
  // Determine current phase based on elapsed time
  if (current_phase == 0 && elapsed_time >= TEMP_PHASES[0].duration) {
    current_phase = 1;
  } else if (current_phase == 1 && elapsed_time >= (TEMP_PHASES[0].duration + TEMP_PHASES[1].duration)) {
    current_phase = 2;
  }
  
  TempPhase phase = TEMP_PHASES[current_phase];
  
  // Temperature-based control with PID-like approach
  float error = phase.target_temp - current_temp;
  
  // Simple proportional control with limits
  int adjustment = (int)(error * 10); // 10 = proportional gain
  int power = phase.pwm_value + adjustment;
  
  // Constrain the output
  if (power < 0) power = 0;
  if (power > 255) power = 255;
  
  return (uint8_t)power;
}

// Temperature measurement with proper conversion
float readTemperature() {
  uint16_t samples[NUM_SAMPLES];
  
  // Sample thermistor multiple times
  for (int i = 0; i < NUM_SAMPLES; i++) {
    samples[i] = analogRead(THERMISTOR_PIN);
    delay(1);
  }

  // Calculate average reading
  uint32_t sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += samples[i];
  }
  
  float average = (float)sum / NUM_SAMPLES;
  
  // Convert to resistance
  float resistance = SERIES_RESISTOR / ((ADC_MAX / average) - 1.0);
  
  // Apply Steinhart-Hart equation (simplified B parameter equation)
  float steinhart = resistance / THERMISTOR_NOMINAL;      // (R/Ro)
  steinhart = log(steinhart);                             // ln(R/Ro)
  steinhart /= BETA_COEFFICIENT;                          // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15);      // + (1/To)
  steinhart = 1.0 / steinhart;                            // Invert
  steinhart -= 273.15;                                    // Convert to Celsius
  
  return steinhart;
}

// LCD display functions
void lcd4rowUpdate() {
  // Format time as MM:SS
  unsigned long seconds = elapsed_time / 1000;
  int minutes = seconds / 60;
  int remainingSeconds = seconds % 60;
  sprintf(time_buffer, "%02d:%02d", minutes, remainingSeconds);
  
  // Format temperature with 1 decimal place
  char temp_buffer[8];
  dtostrf(current_temp, 4, 1, temp_buffer);
  
  // Build menu items
  String menu_items[4] = {
    String(MODES[temperator_mode]),
    String(onScreenPWM),
    String(temp_buffer) + "C",
    String(time_buffer)
  };
  
  // Update LCD
  for (int i = 0; i < 4; i++) {
    lcd.setCursor(0, i);
    
    if (isMenuItemPicked && i == menuItemIndex) {
      // Selected item with brackets
      lcd.print(padString("[" + String(MENU_TITLES[i]) + menu_items[i] + "]"));
    } else if (!isMenuItemPicked && i == menuItemIndex) {
      // Highlighted but not selected item
      lcd.print(padString("-" + String(MENU_TITLES[i]) + menu_items[i]));
    } else {
      // Normal item
      lcd.print(padString(String(MENU_TITLES[i]) + menu_items[i]));
    }
  }
}

String padString(const String& text) {
  int delta = 19 - text.length();
  String padded = text;
  for (int i = 0; i < delta; i++) {
    padded += ' ';
  }
  return padded;
}

// Rotary encoder functions
void rotary_loop() {
  // Check for encoder rotation
  if (rotaryEncoder.encoderChanged()) {
    handle_spin();
  }

  // Check for button press with debounce
  static unsigned long lastButtonPress = 0;
  if (digitalRead(ROTARY_ENCODER_BUTTON_PIN) == LOW) {
    unsigned long now = millis();
    if (now - lastButtonPress > 200) {  // 200ms debounce
      rotary_onButtonClick();
      lastButtonPress = now;
    }
  }
}

void handle_spin() {
  if (isMenuItemPicked) {
    // Editing a specific menu item
    switch (menuItemIndex) {
      case 0:  // Mode
        change_mode();
        break;
      case 1:  // PWM
        change_pwm();
        break;
      case 2:  // Temperature (read-only)
        isMenuItemPicked = false;
        break;
      case 3:  // Time
        change_time();
        break;
    }
  } else {
    // Changing which menu item is selected
    menuItemIndex = rotaryEncoder.readEncoder();
  }
}

void change_mode() {
  temperator_mode = !temperator_mode;
  
  // Reset timers when switching to auto mode
  if (temperator_mode == 1) {
    start_time = millis();
    current_phase = 0;
  }
}

void change_pwm() {
  // Allow larger increments for faster adjustments
  int encoder_value = rotaryEncoder.readEncoder();
  static int last_encoder_value = 0;
  
  int step = 5; // Default step size
  
  // Determine direction and adjust PWM
  if (encoder_value > last_encoder_value) {
    onScreenPWM = min(255, onScreenPWM + step);
  } else if (encoder_value < last_encoder_value) {
    onScreenPWM = max(0, onScreenPWM - step);
  }
  
  last_encoder_value = encoder_value;
}

void change_time() {
  // Reset timer functionality
  start_time = millis();
  current_phase = 0;
}

void rotary_onButtonClick() {
  if (isMenuItemPicked) {
    // Confirm selection
    switch (menuItemIndex) {
      case 0:
        // Mode already changed in change_mode()
        if (temperator_mode == 1) {
          start_time = millis(); // Reset timer when switching to auto
          current_phase = 0;
        }
        break;
      case 1:
        // Apply the PWM setting
        if (temperator_mode == 0) { // Only in manual mode
          temperator_pwm = onScreenPWM;
        }
        break;
      case 2:
        // Read-only
        break;
      case 3:
        // Reset time already done in change_time()
        break;
    }
    isMenuItemPicked = false;
  } else {
    // Enter selection mode
    isMenuItemPicked = true;
  }
}



