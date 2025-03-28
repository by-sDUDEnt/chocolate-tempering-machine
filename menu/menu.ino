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
String menu_arr[4] = {"mode: ", "PWM: ", "Resistance: ", "time: "};
String modes[2] = {"manual", "auto"};
int mode_phase = 0;
int menuItemIndex = 0;



int isMenuItemPicked = false;


int driverPwmPin = 27;
int termistorPin = 33;

// global variables

int PWMrate;
int manualPwmPower;
// LCD init
LiquidCrystal_I2C lcd(0x27, 20, 4);

// encoder init
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);
int temp=0;

String first_menu_vars[4] = {String(modes[mode_phase])+";", String(PWMrate) + ";", String(temp)+ ";", String(millis()/1000)+"s;"};



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
   temp = evaluateResistance(termistorPin);

  // intreal logic based on timings
  
  if (mode_phase == 1){
     PWMrate = getAutoCurrentPower();
  }else{
    PWMrate = manualPwmPower;
  }


  // send pwm + temp
  
  lcdFullscreenUpdate(first_menu_vars);

  // set controller power output 0-255;
  analogWrite(driverPwmPin, PWMrate
); 
  rotary_loop();
  delay(10);
  
}

void lcdFullscreenUpdate(String arr[4]) {

  for (int i=0; i<4;i++){
    lcd.setCursor(0, i);
    // lcd.print(menu_arr[i] + arr[i]);
    if (isMenuItemPicked && i == menuItemIndex){
      lcd.print(print_full_line("["+menu_arr[i] + arr[i]+"]"));
    } else if (!isMenuItemPicked && i == menuItemIndex ){
      lcd.print(print_full_line("-"+menu_arr[i] + arr[i]));
    } else {
      lcd.print(print_full_line(menu_arr[i] + arr[i]));
    }

  }
};

String print_full_line(String text){
  int delta = 19 - text.length();
  for (int i = 0; i < delta; i++) {
        text += ' ';
    };
  return text;
}

int getAutoCurrentPower() {
  unsigned long currentTime = millis();
  
  int MaxPower = 255;
  int LowPower = 10;
  int UpKeepPower = 39;
  int TemperingPower = 9;


  int MaxPowerEndTime = 130;  // end time for interval from start
  int LowPowerTimeEndTime = MaxPowerEndTime + 60;  // end of maxpowertime intetval + interval for lowpowertime
  LowPowerTimeEndTime *= 1000;
  MaxPowerEndTime *= 1000;

  // if (mode_phase == 0){
  //   return TemperingPower;
  // }
  
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
  int resistanceValues[50];
  for (int i = 0; i < 49; i++) {
    resistanceValues[i] = analogRead(pickedTermistor);
    delay(1);
  }

  long sum = 0;
  for (int i = 0; i < 49; i++) {
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
  rotaryEncoder.setBoundaries(0, 3, true);  //minValue, maxValue,  true|false (when max go to min and vice versa)

  //rotaryEncoder.disableAcceleration(); //acceleration is now enabled by default - disable if you dont need it
  rotaryEncoder.setAcceleration(0);  //or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration
  return;
};



void rotary_loop() {
  if (rotaryEncoder.encoderChanged()) {
    handle_spin();
  }


  if (digitalRead(ROTARY_ENCODER_BUTTON_PIN) == LOW) {
    rotary_onButtonClick();
    delay(50);
  }

  // if (rotaryEncoder.isEncoderButtonClicked()) {
  //   rotary_onButtonClick();
  // }
  
}

void handle_spin(){
  if(isMenuItemPicked){
     switch(menuItemIndex){
      case 0:
        change_mode();
        break;
      case 1:
        change_pwm();
        break;
      case 2:
        // no function under resistance
        isMenuItemPicked = false;
        break;
      case 3:
        change_time();
        break;
    
    }
  }else{
    menuItemIndex = rotaryEncoder.readEncoder();;
  }
}


void change_mode(){
  mode_phase = !mode_phase;
}

void change_pwm(){
      manualPwmPower++;
      if (manualPwmPower>=255){
        manualPwmPower=0;
      }
}


void change_time(){
 // idk rest timer?
}






void rotary_onButtonClick() {
  if (isMenuItemPicked){

  }else{
    isMenuItemPicked = true;

    // handle_line_pick();
  }
}



void handle_line_pick(){
  switch (menuItemIndex){
    case 0:
      
    case 1:

      break;
    case 2:

      break;
    case 3:

      break;
  };
}



