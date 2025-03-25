#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "AiEsp32RotaryEncoder.h"
#include "Arduino.h"

#define ROTARY_ENCODER_A_PIN 23
#define ROTARY_ENCODER_B_PIN 18
#define ROTARY_ENCODER_BUTTON_PIN 19
#define ROTARY_ENCODER_VCC_PIN -1
#define ROTARY_ENCODER_STEPS 4

//--------------
// LCD STUFF
LiquidCrystal_I2C lcd(0x27, 20, 4);

AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

int second_phase = 0;

void rotary_onButtonClick() {
  second_phase = 1;
  
}

void rotary_loop() {
  //dont print anything unless value changed
  if (rotaryEncoder.encoderChanged()) {
    // lcd.clear();
    // lcd.setCursor(0, 0);
    // lcd.print(rotaryEncoder.readEncoder());
  }
  if (rotaryEncoder.isEncoderButtonClicked()) {
    rotary_onButtonClick();
  }
}

void IRAM_ATTR readEncoderISR() {
  rotaryEncoder.readEncoder_ISR();
}

// END LCD STUF
//-------------




int RPWM_Output = 27;
int termistorPin = 33;
int buttonPin = 35;

// from 0 to 255 s
int PMWHeatPowerLevel = 0;
int targetTemp = 2650;  // about 45C
int workMode = 0;
int termistorResistance;
int maxPowerTimeInSeconds;

// flags to transition to debug values
int debugMode = 0;

int DebugTargetTemp;
int DebugPWMHeatPowerLevel;


// menu variables

bool heatModuleIsOn = false;
bool climaxFlag = false;
// 0 -  startup, heating off
// 1 - rapid heating at max power
// 2 - first heating phase, up to reaching targeted
// 3 - upkeep mode for targeted
// 4 - cooling for temperation technique, for future develompent




const byte numChars = 32;
char receivedChars[numChars];  // an array to store the received data

boolean newData = false;


void setup()

{

//---------
  // LCD SETUP
  Serial.begin(115200);

  //we must initialize rotary encoder
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  //set boundaries and if values should cycle or not
  //in this example we will set possible values between 0 and 1000;
  bool circleValues = true;
  rotaryEncoder.setBoundaries(0, 7, circleValues);  //minValue, maxValue, circleValues true|false (when max go to min and vice versa)

  /*Rotary acceleration introduced 25.2.2021.
   * in case range to select is huge, for example - select a value between 0 and 1000 and we want 785
   * without accelerateion you need long time to get to that number
   * Using acceleration, faster you turn, faster will the value raise.
   * For fine tuning slow down.
   */
  //rotaryEncoder.disableAcceleration(); //acceleration is now enabled by default - disable if you dont need it
  rotaryEncoder.setAcceleration(250);  //or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration
  lcd.init();
  lcd.backlight();


  // END LCD SETUP
  //-----


  pinMode(RPWM_Output, OUTPUT);  // heating resistor driver
  pinMode(termistorPin, INPUT);  // termistor input after voltage divider
  pinMode(buttonPin, INPUT);     // physical button for disabling heating
  pinMode(LED_BUILTIN, OUTPUT);  // status LED output
}


void loop() {

  // LCD STUFF
  if (Serial.available()) {
    delay(100);
    lcd.clear();
    while (Serial.available() > 0) {
      lcd.write(Serial.read());
    }
  }
  rotary_loop();
  // LCD STUF

  // lcd.clear();
  int temp = evaluateResistance(termistorPin);

  recvWithEndMarker();
  showNewData();




  // upkeeppowertime is infinite, because its upkeep



  int pwmPower = getCurrentPower();

  lcd.setCursor(0, 1);
 
  lcd.print("PWM: " + String(pwmPower)+ "   ");

  lcd.setCursor(0, 2);
  lcd.print("Resistance: " + String(temp));

  lcd.setCursor(0, 3);
  lcd.print("time: " + String(millis()/1000)+"s");

  
 

  analogWrite(RPWM_Output, pwmPower); // enable for loop

  // just for testing
  // analogWrite(RPWM_Output, 20);


  delay(1000);
  
}


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







void startUp_loop() {
  if (heatModuleIsOn == true) {
    workMode = 1;  // transition into 1st reachClimaxmode
  }
}


void reachClimax(int flag) {
  // i need timer here?
  if (flag == false) {
    // climax starterd, flag set to true

    climaxFlag, flag == true;
  }
}


void reachingTeamerature_loop() {
  // set resistor`s PWM to variable (from 0 to 255);
  if (PMWHeatPowerLevel > 255) {
    PMWHeatPowerLevel = 255;
    Serial.println("Max power level is 255!");
  } else if (PMWHeatPowerLevel < 0) {
    PMWHeatPowerLevel = 0;
    Serial.println("Min power level is 0!");
  } 

  analogWrite(RPWM_Output, PMWHeatPowerLevel);
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


void coolOff_loop() {
}

void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    } else {
      receivedChars[ndx] = '\0';  // terminate the string
      ndx = 0;
      newData = true;
    }
  }
}

void showNewData() {
  if (newData == true) {
    Serial.print("This just in ... ");
    Serial.println(receivedChars);
    newData = false;
  }
}
