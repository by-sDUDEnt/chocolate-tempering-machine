#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "AiEsp32RotaryEncoder.h"
#include "Arduino.h"

#define ROTARY_ENCODER_A_PIN 23
#define ROTARY_ENCODER_B_PIN 18
#define ROTARY_ENCODER_BUTTON_PIN 19
#define ROTARY_ENCODER_VCC_PIN -1
#define ROTARY_ENCODER_STEPS 4


// pins on esp32devkit
int driverPwmPin = 27;
int termistorPin = 33;

// global variables
int second_phase = 0;

// LCD init
LiquidCrystal_I2C lcd(0x27, 20, 4);

// encoder init
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);






void setup(){

  Serial.begin(115200);

  lcd_setup();
  encoder_setup();

  pinMode(driverPwmPin, OUTPUT);  // heating resistor driver
  pinMode(termistorPin, INPUT);  // termistor input after voltage divider
  pinMode(LED_BUILTIN, OUTPUT);  // status LED output
}


void loop() {
  // probe thermistor 1000th times and gets average temp;
  int temp = evaluateResistance(termistorPin);

  // intreal logic based on timings
  int pwmPower = getCurrentPower();

  // send pwm + temp
  lcdFullscreenUpdate(pwmPower, temp);

  // set controller power output 0-255;
  analogWrite(driverPwmPin, pwmPower); 

  delay(1000);
  
}

void lcdFullscreenUpdate(int pwmPower, int temp) {
  lcd.setCursor(0, 1);
  lcd.print("PWM: " + String(pwmPower)+ "   ");

  lcd.setCursor(0, 2);
  lcd.print("Resistance: " + String(temp));

  lcd.setCursor(0, 3);
  lcd.print("time: " + String(millis()/1000)+"s");
  return;
};


int getCurrentPower() {
  unsigned long currentTime = millis();
  
  int MaxPower = 255;
  int LowPower = 10;
  int UpKeepPower = 39;
  int TemperingPower = 9;


  int MaxPowerEndTime = 130;  // end time for interval from start
  int LowPowerTimeEndTime = MaxPowerEndTime + 60;  // end of maxpowertime intetval + interval for lowpowertime
  LowPowerTimeEndTime *= 1000;
  MaxPowerEndTime *= 1000;

  if (second_phase == 1){
    return TemperingPower;
  }
  
  if (currentTime < MaxPowerEndTime){
    return MaxPower;
  }

  if (currentTime < LowPowerTimeEndTime){
    return LowPower;
  }

  return UpKeepPower;

}


// get average value of resistance, 100 probe, 1 per 1mlsecond
int evaluateResistance(int pickedTermistor) {
  int resistanceValues[1000];
  for (int i = 0; i < 1000; i++) {
    resistanceValues[i] = analogRead(pickedTermistor);
    delay(1);
  }

  long sum = 0;
  for (int i = 0; i < 1000; i++) {
    sum += resistanceValues[i];
  }

  // Calculate the average
  float average = (float)sum / 1000;

  // Print the result
  
  return (int)average;
}



void lcd_setup(){
  lcd.init();
  lcd.backlight();
}


void IRAM_ATTR readEncoderISR() {
  rotaryEncoder.readEncoder_ISR();
};

void encoder_setup() {
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  rotaryEncoder.setBoundaries(0, 7, true);  //minValue, maxValue,  true|false (when max go to min and vice versa)

  //rotaryEncoder.disableAcceleration(); //acceleration is now enabled by default - disable if you dont need it
  rotaryEncoder.setAcceleration(250);  //or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration
  return;
};



void rotary_loop() {
  //dont print anything unless value changed
  if (rotaryEncoder.encoderChanged()) {
      // nothing
  }
  if (rotaryEncoder.isEncoderButtonClicked()) {
    rotary_onButtonClick();
  }
}


void rotary_onButtonClick() {
  second_phase = 1;
}



