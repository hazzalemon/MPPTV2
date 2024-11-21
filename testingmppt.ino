
#include <EEPROM.h>  //SYSTEM PARAMETER  - EEPROM Library (By: Arduino)
#include "arduino.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"


Adafruit_ADS1015 ads1015;
Adafruit_ADS1015 ads1015_B;
TaskHandle_t Core2;

#define SDA_PIN 36
#define SCL_PIN 37

int pwmMaxLimited = 1800;
int pwmMax = 2000,
  update_count = 0,
  test = 0,
    conv1 = 0,          // SYSTEM PARAMETER -
  conv2 = 0,            // SYSTEM PARAMETER -
  errorCount = 0,       // SYSTEM PARAMETER -
  errorCountLimit = 5,  //  USER PARAMETER - Maximum number of errors

  avgCountVS = 6,  //  CALIB PARAMETER - Voltage Sensor Average Sampling Count (Recommended: 3)
  avgCountCS = 6,  //  CALIB PARAMETER - Current Sensor Average Sampling Count (Recommended: 4)
  avgCountTS = 500,
    errorTimeLimit = 1000,  //  USER PARAMETER - Time interval for reseting error counter (milliseconds)
  ERR = 0,
  phaseA_trim = 0,
  PPWM = 0,
  PWM = 0;  // SYSTEM PARAMETER



bool
  prev_change = 0,
  buck_mode = 0,
  prev_buck_mode = 0,
  bypassEnable = 0,  // SYSTEM PARAMETER -

  MPPT_Mode = 1,             //   USER PARAMETER - Enable MPPT algorithm, when disabled charger uses CC-CV algorithm
  output_Mode = 1,           //   USER PARAMETER - 0 = PSU MODE, 1 = Charger Mode
  disableFlashAutoLoad = 0,  //   USER PARAMETER - Forces the MPPT to not use flash saved settings, enabling this "1" defaults to programmed firmware settings.
  enablePPWM = 1,            //   USER PARAMETER - Enables Predictive PWM, this accelerates regulation speed (only applicable for battery charging application)
  enableWiFi = 1,            //   USER PARAMETER - Enable WiFi Connection
  enableFan = 1,             //   USER PARAMETER - Enable Cooling Fan
  enableBluetooth = 1,       //   USER PARAMETER - Enable Bluetooth Connection
  enableLCD = 1,             //   USER PARAMETER - Enable LCD display
  enableLCDBacklight = 1,    //   USER PARAMETER - Enable LCD display's backlight
  chargingPause = 0,         // SYSTEM PARAMETER -
  REC = 0,                   // SYSTEM PARAMETER -

  overrideFan = 0,  //   USER PARAMETER - Fan always on
  enableDynamicCooling = 0,
  BNC = 0,  // SYSTEM PARAMETER -
  FLV = 0,  // SYSTEM PARAMETER -
  IUV = 0,  // SYSTEM PARAMETER -
  IOV = 0,  // SYSTEM PARAMETER -
  IOC = 0,  // SYSTEM PARAMETER -
  OUV = 0,  // SYSTEM PARAMETER -
  OOV = 0,  // SYSTEM PARAMETER -
  OOC = 0,  // SYSTEM PARAMETER -
  OTE = 0;  // SYSTEM PARAMETER -

double
  output_volt = 0,
  vin = 0,
  vout = 0,
  cin = 0,
  cout = 0,
  c_a = 0,
  c_b = 0,
  prev_pwr = 0,
  prev_duty = 0,
  duty = 0,
  in_pwr = 0,
  powerInput = 0.0000,        // SYSTEM PARAMETER - Input power (solar power) in Watts
  powerInputPrev = 0.0000,    // SYSTEM PARAMETER - Previously stored input power variable for MPPT algorithm (Watts)
  powerOutput = 0.0000,       // SYSTEM PARAMETER - Output power (battery or charing power in Watts)
  energySavings = 0.0000,     // SYSTEM PARAMETER - Energy savings in fiat currency (Peso, USD, Euros or etc...)
  voltageInput = 0.0000,      // SYSTEM PARAMETER - Input voltage (solar voltage)
  voltageInputPrev = 0.0000,  // SYSTEM PARAMETER - Previously stored input voltage variable for MPPT algorithm
  voltageOutput = 0.0000,     // SYSTEM PARAMETER - Input voltage (battery voltage)
  currentInput = 0.0000,      // SYSTEM PARAMETER - Output power (battery or charing voltage)
  currentOutput = 0.0000,
  mppt = 0;

float
  voltageBatteryMax = 12.6000,  //   USER PARAMETER - Maximum Battery Charging Voltage (Output V)
  voltageBatteryMin = 5.000,    //   USER PARAMETER - Minimum Battery Charging Voltage (Output V)
  loopTime = 0.0000,
  voltageDropout = 1.0000,  //  CALIB PARAMETER - Buck regulator's dropout voltage (DOV is present due to Max Duty Cycle Limit)
  currentCharging = 15.0000,
  PPWM_margin = 199.5000,         //  CALIB PARAMETER - Minimum Operating Duty Cycle for Predictive PWM (%)
  PWM_MaxDC = 10.0000,            //   USER PARAMETER - Maximum Charging Current (A - Output)
  outputDeviation = 0.0000,       // SYSTEM PARAMETER - Output Voltage Deviation (%)
  vInSystemMin = 7.000,           //  CALIB PARAMETER -
  voltageBatteryThresh = 1.0000,  //  CALIB PARAMETER - Power cuts-off when this voltage is reached (Output V)
  currentOutAbsolute = 50.0000,
  currentInAbsolute = 31.0000,  //  CALIB PARAMETER - Maximum Input Current The System Can Handle (A - Input)

  efficiencyRate = 1.0000,   //  CALIB PARAMETER - Theroretical Buck Efficiency (% decimal)
  electricalPrice = 9.5000;  //   USER PARAMETER - Input electrical price per kWh (Dollar/kWh,Euro/kWh,Peso/kWh)

unsigned long
  currentErrorMillis = 0,     //SYSTEM PARAMETER -
  currentButtonMillis = 0,    //SYSTEM PARAMETER -
  currentSerialMillis = 0,    //SYSTEM PARAMETER -
  currentRoutineMillis = 0,   //SYSTEM PARAMETER -
  currentLCDMillis = 0,       //SYSTEM PARAMETER -
  currentLCDBackLMillis = 0,  //SYSTEM PARAMETER -
  currentWiFiMillis = 0,      //SYSTEM PARAMETER -
  currentMenuSetMillis = 0,   //SblynkYSTEM PARAMETER -
  prevButtonMillis = 0,       //SYSTEM PARAMETER -
  prevSerialMillis = 0,       //SYSTEM PARAMETER -
  prevRoutineMillis = 0,      //SYSTEM PARAMETER -
  prevErrorMillis = 0,        //SYSTEM PARAMETER -
  prevWiFiMillis = 0,         //SYSTEM PARAMETER -
  prevLCDMillis = 0,          //SYSTEM PARAMETER -
  prevLCDBackLMillis = 0,     //SYSTEM PARAMETER -
  timeOn = 0,                 //SYSTEM PARAMETER -
  loopTimeStart = 0,          //SYSTEM PARAMETER - Used for the loop cycle stop watch, records the loop start time
  loopTimeEnd = 0,            //SYSTEM PARAMETER - Used for the loop cycle stop watch, records the loop end time
  secondsElapsed = 0;         //SYSTEM PARAMETER -

void coreTwo(void* pvParameters) {
  setup_display();
  while (1) {
    update_display();
  }
}
void setup() {
  MCPWM_SetUP();
  Serial.begin(57600);
  setup_display();
  // xTaskCreatePinnedToCore(coreTwo,"coreTwo",10000,NULL,0,&Core2,0);
  Wire1.begin(SDA_PIN, SCL_PIN);
  ads1015.begin(72, &Wire1);
  ads1015_B.begin(73, &Wire1);

  // Serial.println("setup MCPWM!");
}

void loop() {
  read_sensors();
  // delay(100);
  Device_Protection();
  Charging_Algorithm();
  update_display();
  Serial.println((String) "loop time: " + loopTime );
  update_count++;
  if (update_count > 10){
    update_count = 0;
    if (c_a > c_b + 0.2){
      phaseA_trim--;
    }else if (c_a < c_b - 0.2){
      phaseA_trim++;
    }
  }
  Serial.println((String)"PLoss: " + (powerInput - powerOutput));
  // Serial.println((String)"input voltage: "+ vin + ", output voltage: "+ vout + ", input current: "+ cin + ", output_current: " + cout + ", phase_A current: " + c_b + ", phase_b current: "+ c_a);
  // Serial.println((String)"duty: " + test + "in_pwr: " + in_pwr);
  // // update_display();
  //   if(Serial.available()>0)
  // {
  //   char incomingByte = Serial.read();
  //   test = incomingByte - '0';
  //   Serial.println(test);
  // }
  // if(test > 100){
  // Set_boost_PWM(float(200-test));
  // }
  // else if(test == 100){
  //    Set_buck_PWM(float(99.9));
  // }
  // else{
  //   Set_buck_PWM(float(test));
  // }
}