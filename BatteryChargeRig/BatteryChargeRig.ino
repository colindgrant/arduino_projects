/*
  Colin's battery charge/test rig
  Trinket ATtiny85 5v
  INA219 current monitor
  TP4056 single cell lithium ion charger
  TCA9548A I2C multiplexer
  ADS1015 voltage monitor
  20x4 LCD w/I2C backpack
*/

#include <TinyWireM.h>          // Adafruit Wwire library for ATTiny85 (ie Trinket), which uses USI to provide I2C or SPI
#include <USI_TWI_Master.h>     // Also included by TinyWireM, so not strictly necessary

// These I2C device librarys must be included *after* TinyWireM
#include <LiquidCrystal_I2C.h>  // LCD display with I2C port expander backpack
#include "Adafruit_INA219.h"    // Adafruit's INA219 current/voltage monitor library modified to use TinyWireM, see: https://github.com/josefadamcik/INA219PowerMeasurementUnit
#include <Adafruit_ADS1015.h>   // Adafruit's ADC breakout as voltage monitor

// This project assumes 12-bit ADS1015 chip by default
// Uncomment if using the higher precision 16-bit ADS1115 chip
#define ADS1115

// ********** Device objects **********

/* The I2C 20x4 LCD address is 0x27 by default. Pulling down A0, A1, A2 gives lower addresses:
  A0 A1 A2 HEX
  1 1 1 0x27
  0 1 1 0x26
  1 0 1 0x25
  0 0 1 0x24
  1 1 0 0x23
  0 1 0 0x22
  1 0 0 0x21
  0 0 0 0x20

  LiquidCrystal_I2C screen_0(0x27, 20, 4); // default
  LiquidCrystal_I2C screen_1(0x26, 20, 4); // A0 low
  LiquidCrystal_I2C screen_2(0x25, 20, 4); // A1 low
  LiquidCrystal_I2C screen_3(0x24, 20, 4); // A0 and A1 low
  ...
*/
LiquidCrystal_I2C screens[] = {LiquidCrystal_I2C(0x27, 20, 4), LiquidCrystal_I2C(0x26, 20, 4)};

// Define all 4 INA219 I2C addresses (not all INA219 physical modules), put them in an array
Adafruit_INA219 meters[] = {Adafruit_INA219(0x40), Adafruit_INA219(0x41), Adafruit_INA219(0x44), Adafruit_INA219(0x45)};

// ADC used as voltage monitor, I2C address 0x48 HEX, 72 DEC
#ifdef ADS1115
  Adafruit_ADS1115 ads;
#else
  Adafruit_ADS1015 ads;
#endif


// ********** Global Constants **********

// The I2C address of the I2C multiplexer
#define TCAADDR 0x70

// Number of devices (and thus their objects) are fixed, so just define them
// for later iterating without needing to calculate them from an array size
const uint8_t numMeters = 4;
const uint8_t numScreens = 2;
const uint8_t numBatteryModules = 4;

#ifdef ADS1115
  const float voltsPerDivision = .0001875;
#else
  const float voltsPerDivision = .003;
#endif

// ********** Global Variables **********

// Used to iterate when the loop() repeats
// E.g. 3 modules have indexes [0,1,2], start at index 0
uint8_t currentBatteryModule = 0;


// ********** Functions **********

void tcaselect(uint8_t i) {
  if (i > 7) return;

  TinyWireM.beginTransmission(TCAADDR);
  TinyWireM.write(1 << i);
  TinyWireM.endTransmission();
  delay(50); // just in case there is some switching time required
}

void setup(void)
{
  uint8_t index; // common loop iterator

  // These are not strictly necessary, call them anyway
  TinyWireM.begin(); // Modified INA219 begin() would also do this
  ads.begin(); // Calls Wire.begin(); perhaps it too can be converted to use TinyWireM?

  // Initialize all 4 INA219 objects in memory
  // Once up and running, groups of 4 physical INA219 modules will be swapped
  // under these objects by the I2C multiplexer, but they won't know.
  //
  // Note this also calls `TinyWireM.begin();` to initialize the I2C bus
  for (index = 0; index < numMeters; index++) {
    meters[index].begin();
    meters[index].setCalibration_32V_1A();
  }

  // Initialize the LCD screens
  for (index = 0; index < numScreens; index++) {
    screens[index].init();  //initialize the lcd
    screens[index].backlight();  //open the backlight
  }
}

void loop(void)
{

  uint8_t currentScreen;
  uint8_t lineNumber; // 'top' LCD line for the currentBatteryModule
  uint8_t error;
  uint8_t index; // common loop iterator
  float voltage;

  // Iterate the I2C multiplexer to connect the master bus to the battery module for this round of the loop
  if (currentBatteryModule == numBatteryModules) {
    currentBatteryModule = 0;
  }
  tcaselect(currentBatteryModule);

  // Set up where things will be printed for the currentBatteryModule
  currentScreen = (currentBatteryModule / 2); // take advantage of integer math, truncating the remainder
  lineNumber = (currentBatteryModule % 2) * 2;

  // Both batttery modules types show voltage, so print the 'v' character at the end of the voltage line
  screens[currentScreen].setCursor (19, lineNumber);
  screens[currentScreen].print("v");

  // Find out which type of battery module is connected to the currently selected I2C bus:
  // 1) Fixed current, voltage monitoring only, via ADS1015 ADC on I2C 0x48 HEX, 72 DEC
  // 2) Adjustable current, via 4 modified TP4056 chargers and INA219 power monitors
  TinyWireM.beginTransmission(0x48);
  error = TinyWireM.endTransmission();
  if (error == 0) {
    // ADS1015 found on the currently attached I2C bus, collect voltage only
    for (index = 0; index < numMeters; index++) {
      voltage = (ads.readADC_SingleEnded(index) * voltsPerDivision);
      screens[currentScreen].setCursor (index * 5, lineNumber);
      screens[currentScreen].print(voltage, 2);
      screens[currentScreen].setCursor (0, lineNumber + 1);
      screens[currentScreen].print("^ Fixed 500mA max. ^");
    }
  } else {
    // Currently attached I2C bus has INA219 modules, collecting voltage and current
    int16_t current_mA;
    
    // INA219 equipped battery modules show current, so print the 'i' character at the end of the current line
    screens[currentScreen].setCursor (19, lineNumber + 1);
    screens[currentScreen].print("i");
  
    // Iterate the INA219 modules on the currentBatteryModule
    for (index = 0; index < numMeters; index++) {
      voltage = meters[index].getBusVoltage_V();
      current_mA = (int16_t) (meters[index].getCurrent_mA() + .5);

      screens[currentScreen].setCursor ((index * 5), lineNumber);
      screens[currentScreen].print(voltage, 2);
      screens[currentScreen].setCursor ((index * 5), lineNumber + 1);
      if (current_mA < 2) {
        current_mA = 0; // don't show random 1 mA readings
      }
      if (current_mA < 10) {
        screens[currentScreen].print("   ");
      } else if (current_mA < 100) {
        screens[currentScreen].print("  ");
      } else if (current_mA < 1000) {
        screens[currentScreen].print(" ");
      }
      screens[currentScreen].print(current_mA);
    }
  }

  delay(100);
  currentBatteryModule++;
}
