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
const int driverPwmPin = 27;  // Heating resistor driver
const int termistorPin = 33;  // Thermistor input after voltage divider

// Menu system
const String menu_titles[4] = {"mode: ", "PWM: ", "Resistance: ", "time: "};
const String modes[2] = {"manual", "auto"};

// Device settings
String temperator_setings_mode = modes[0];  // Default to manual mode
int temperator_setings_pwm = 0;
int temperator_setings_time = 0;

// Menu state
int mode_phase = 0;
int menuItemIndex = 0;
bool isMenuItemPicked = false;
int onScreenPWM = 0;
int selectedManualPWM = 0;
int temp = 0;
String onScreenTime = "0";

String menu_args[4] = {String(modes[mode_phase]), String(onScreenPWM), String(temp), String(onScreenTime)};

// Hardware initialization
LiquidCrystal_I2C lcd(0x27, 20, 4);  // LCD with I2C address 0x27, 20 columns, 4 rows
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(
  ROTARY_ENCODER_A_PIN, 
  ROTARY_ENCODER_B_PIN, 
  ROTARY_ENCODER_BUTTON_PIN, 
  ROTARY_ENCODER_VCC_PIN, 
  ROTARY_ENCODER_STEPS
);

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
  pinMode(driverPwmPin, OUTPUT);  // Heating resistor driver
  pinMode(termistorPin, INPUT);   // Thermistor input
  pinMode(LED_BUILTIN, OUTPUT);   // Status LED
}

void loop() {
  // Read temperature by measuring thermistor resistance
  temp = evaluateResistance(termistorPin);
  
  // Update LCD display
  lcd4rowUpdate(menu_args);
  
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

// Temperature control functions
void handleTemperator() {
  if (temperator_setings_mode == "manual") {
    changeTemperatorPWM(onScreenPWM);  // Use the onScreenPWM value in manual mode
  } else if (temperator_setings_mode == "auto") {
    changeTemperatorPWM(getAutoCurrentPower());
  }

  // Apply the PWM value to the driver
  analogWrite(driverPwmPin, temperator_setings_pwm); 
}

void changeTemperatorPWM(int pwm) {
  temperator_setings_pwm = pwm;
}

int getAutoCurrentPower() {
  unsigned long currentTime = millis();
  
  // Power levels for different stages
  const int MaxPower = 255;
  const int LowPower = 10;
  const int UpKeepPower = 39;
  
  // Timing thresholds in milliseconds
  const unsigned long MaxPowerEndTime = 130 * 1000;  // 130 seconds
  const unsigned long LowPowerTimeEndTime = MaxPowerEndTime + (60 * 1000);  // 190 seconds
  
  if (currentTime < MaxPowerEndTime) {
    return MaxPower;
  } else if (currentTime < LowPowerTimeEndTime) {
    return LowPower;
  } else {
    return UpKeepPower;
  }
}

// Temperature sensing
int evaluateResistance(int pickedTermistor) {
  int resistanceValues[50];
  
  // Sample thermistor 50 times
  for (int i = 0; i < 49; i++) {
    resistanceValues[i] = analogRead(pickedTermistor);
    delay(1);
  }

  // Calculate average reading
  long sum = 0;
  for (int i = 0; i < 49; i++) {
    sum += resistanceValues[i];
  }
  
  float average = (float)sum / 1000;
  
  return (int)average;
}

// LCD display functions
void lcd4rowUpdate(String arr[4]) {
  for (int i = 0; i < 4; i++) {
    lcd.setCursor(0, i);
    
    if (isMenuItemPicked && i == menuItemIndex) {
      // Selected item with brackets
      lcd.print(print_full_line("[" + menu_titles[i] + arr[i] + "]"));
    } else if (!isMenuItemPicked && i == menuItemIndex) {
      // Highlighted but not selected item
      lcd.print(print_full_line("-" + menu_titles[i] + arr[i]));
    } else {
      // Normal item
      lcd.print(print_full_line(menu_titles[i] + arr[i]));
    }
  }
}

String print_full_line(String text) {
  int delta = 19 - text.length();
  for (int i = 0; i < delta; i++) {
    text += ' ';
  }
  return text;
}

// Rotary encoder functions
void rotary_loop() {
  // Check for encoder rotation
  if (rotaryEncoder.encoderChanged()) {
    handle_spin();
  }

  // Check for button press
  if (digitalRead(ROTARY_ENCODER_BUTTON_PIN) == LOW) {
    rotary_onButtonClick();
    delay(50);  // Debounce
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
      case 2:  // Resistance (read-only)
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
  mode_phase = !mode_phase;
  menu_args[0] = modes[mode_phase];
}

void change_pwm() {
  onScreenPWM++;
  if (onScreenPWM >= 255) {
    onScreenPWM = 0;
  }
  menu_args[1] = String(onScreenPWM);
}

void change_time() {
  // Not implemented yet
  menu_args[3] = String(millis() / 1000);
}

void rotary_onButtonClick() {
  if (isMenuItemPicked) {
    // Confirm selection
    switch (menuItemIndex) {
      case 0:
        temperator_setings_mode = modes[mode_phase];
        break;
      case 1:
        temperator_setings_pwm = onScreenPWM;
        break;
      case 2:
        // Read-only
        break;
      case 3:
        // Reset time (not implemented)
        break;
    }
    isMenuItemPicked = false;
  } else {
    // Enter selection mode
    isMenuItemPicked = true;
  }
}



