/*
  Colin's battery charge/test rig
  Trinket ATtiny85 5v
  INA219 current monitor
  TP4056 single cell lithium ion charger
  TCA9548A I2C multiplexer
  ADS1015 voltage monitor
  20x4 LCD w/I2C backpack
*/

#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

//#define SERIALON

//// Define all 4 INA219 I2C addresses (not all INA219 physical modules), put them in an array
Adafruit_INA219 meters[] = {Adafruit_INA219(0x40), Adafruit_INA219(0x41), Adafruit_INA219(0x44), Adafruit_INA219(0x45)};

//  int numMeters = sizeof(meters)/sizeof(meters[0]);
//  The size will always be 4, so just set it to save calculating it
uint8_t numMeters = 4;

uint8_t numBatteryModules = 3;
uint8_t currentBatteryModule = 0; // start here [0,1,2] = 3 modules

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
  LiquidCrystal_I2C screen_2(0x25, 20, 4); //A1 low
  LiquidCrystal_I2C screen_3(0x24, 20, 4); // A0 and A1 low
  ...
*/
LiquidCrystal_I2C screens[] = {LiquidCrystal_I2C(0x27, 20, 4), LiquidCrystal_I2C(0x26, 20, 4)};

#define TCAADDR 0x70

void tcaselect(uint8_t i) {
#ifdef SERIALON
  Serial.print("Selecting multiplexer port: ");
  Serial.print(i);
  Serial.print("...");
#endif

  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}


void iic_discover()
{
#ifdef SERIALON
  byte error, address, nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknow error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
  delay(5000); // wait 5 seconds for next scan
#endif
}

void setup(void)
{

#ifdef SERIALON
  Serial.begin(115200);
  while (!Serial) {
    // will pause Zero, Leonardo, etc until serial console opens
    delay(1);
  }
  Serial.println("Hello!");
#endif


  // Setup I2C multiplexer on the 0 group of INA219 monitors
  Wire.begin();
  uint8_t t = 0;
  tcaselect(t);
  delay(1000);

  // needs serial monitor
  //  iic_discover();

  // Setup I2C LCD 0
  screens[0].init();  //initialize the lcd
  screens[0].backlight();  //open the backlight
  screens[0].setCursor (0, 0);
  screens[0].print("This is LCD screen ");
  screens[0].print("0");

  // Setup I2C LCD 1
  screens[1].init();  //initialize the lcd
  screens[1].backlight();  //open the backlight
  screens[1].setCursor (0, 0);
  screens[1].print("This is LCD screen ");
  screens[1].print("1");

  // Setup INA219s
  uint32_t currentFrequency;

  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  //ina219.begin();
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  //ina219.setCalibration_16V_400mA();


  // Initialize all 4 INA219 objects in memory
  // Once up and running, groups of 4 physical INA219 modules will be swapped
  // under these objects by the I2C multiplexer, but they won't know.
  for (uint8_t meter = 0; meter < numMeters; meter++) {
    // put init status on screen 0
    screens[0].setCursor (0, meter);
    screens[0].print("Initializing meter ");
    screens[0].print(meter);

    meters[meter].begin();
    meters[meter].setCalibration_32V_1A();
  }


  delay(1000);
  //  Serial.println("Measuring voltage and current with INA219_A ...");
  screens[0].clear();
  screens[1].clear();
}

void loop(void)
{

  //  char paddedCurrent[4];
  short paddedCurrent;
  short currentScreen;


  //  Serial.println();
  //  Serial.println("Fresh loop");
  //
  //  Serial.print("Battery Module: "); Serial.println(currentBatteryModule);

  //  delay(1000);

  // iterate the I2C multiplexer to select a battery module
  if (currentBatteryModule == numBatteryModules) {
    //    Serial.println("Setting module to 0");
    currentBatteryModule = 0;
  }
  //  Serial.print("Selecting bus: "); Serial.println(currentBatteryModule);
  tcaselect(currentBatteryModule);
  //  Serial.print("Battery Module: "); Serial.println(currentBatteryModule);

  currentScreen = (currentBatteryModule / 2); // take advantage of integer math, truncating the remainder

  //  //  lcd.clear();
  screens[currentScreen].setCursor (19, 0);
  screens[currentScreen].print("v");
  screens[currentScreen].setCursor (19, 1);
  screens[currentScreen].print("i");
  screens[currentScreen].setCursor (19, 2);
  screens[currentScreen].print("v");
  screens[currentScreen].setCursor (19, 3);
  screens[currentScreen].print("i");

  //  // identify battery module
  //  screens[currentScreen].setCursor (0, (currentBatteryModule % 2) * 2);
  //  screens[currentScreen].print (currentBatteryModule);

  // now iterate the INA219 modules on the currently selected battery module
  for (uint8_t meter = 0; meter < numMeters; meter++) {
    //     Serial.print("In loop Battery Module: "); Serial.println(currentBatteryModule);


    //    float shuntvoltage = 0;
    float busvoltage = 0;
    float current_mA = 0;
    //    float loadvoltage = 0;
    //    float power_mW = 0;

    //    shuntvoltage = meters[meter].getShuntVoltage_mV();
    busvoltage = meters[meter].getBusVoltage_V();
    current_mA = meters[meter].getCurrent_mA();
    //    power_mW = meters[meter].getPower_mW();
    //    loadvoltage = busvoltage + (shuntvoltage / 1000);

    if (current_mA < 1) {
      //      sprintf(paddedCurrent, "% 4d", 0);
      paddedCurrent = 0;
    } else {
      //      sprintf(paddedCurrent, "% 4d", round(current_mA));
      paddedCurrent = round(current_mA);
    }

    //    paddedCurrent = round(current_mA);

#ifdef SERIALON
    Serial.print("Meter "); Serial.print(meter + 1); Serial.println(":");
    Serial.print(" Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
    //    Serial.print(" Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
    //    Serial.print(" Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
    Serial.print(" Current:       "); Serial.print(current_mA); Serial.println(" mA");
    //    Serial.print(" Power:         "); Serial.print(power_mW); Serial.println(" mW");
    //    Serial.println("");

    //    int roundedCurrent = round(current_mA);
    //    Serial.print(" Rounded:       "); Serial.print(roundedCurrent); Serial.println(" mA");
    //    Serial.println(paddedCurrent);
#endif

    screens[currentScreen].setCursor ((meter * 5), (currentBatteryModule % 2) * 2);
    screens[currentScreen].print(busvoltage, 2); //screens[currentScreen].print(" ");
    screens[currentScreen].setCursor ((meter * 5), ( ((currentBatteryModule % 2) * 2) + 1) );
    if (paddedCurrent < 10) {
      screens[currentScreen].print("   ");
    } else if (paddedCurrent < 100) {
      screens[currentScreen].print("  ");
    } else if (paddedCurrent < 1000) {
      screens[currentScreen].print(" ");
    }
    screens[currentScreen].print(paddedCurrent);
    //    screens[currentScreen].setCursor ( 0, 2 );            // go to the third row
    //    screens[currentScreen].print("bus: ");
    //    screens[currentScreen].print(currentBatteryModule);
    //    screens[currentScreen].setCursor ( 0, 3 );            // go to the fourth row
    //    screens[currentScreen].print(millis());


  }

  delay(200);
  //  Serial.print("Battery Module before: "); Serial.println(currentBatteryModule);
  currentBatteryModule++;
  //  Serial.print("Battery Module after: "); Serial.println(currentBatteryModule);
}
