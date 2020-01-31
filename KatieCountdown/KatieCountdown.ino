#include <Wire.h>                 // I2C library
#include <LiquidCrystal.h>        // LCD display library, non-I2C
#include <RTClib.h>               // DS1307 Real Time Clock library
#include <Adafruit_GFX.h>         // Core graphics library
#include <Adafruit_HX8357.h>      // Hardware-specific TFT library
#include <SdFat.h>                // SD card & FAT filesystem library
#include <Adafruit_ImageReader.h> // Image-reading functions
#include <Adafruit_STMPE610.h>    // Touchscreen controller


// TFT/SD pins over and above default SPI pins for MOSI, MISO, CLK
#define TFT_CS   10 // Marked CS on board, Trinket pinout calls pin 10 SPI 'SS' 
#define TFT_DC   A0 // Marked D/C on board
#define SD_CS    A3 // Marked CCS on board

#define DEBUG true // flag to turn on/off debugging
#define Serial if(DEBUG)Serial

// Initialize device objects
SdFat                 sdfs;         // SD card filesystem
Adafruit_ImageReader  reader(sdfs); // Image-reader object, pass in SD filesys
Adafruit_HX8357       tft(TFT_CS, TFT_DC);
LiquidCrystal         lcd(9, 8, 6, 5, 4, 3);
RTC_DS1307            rtc;
Adafruit_STMPE610     touch;

// Define constants
const byte numDisplayModes = 5;
//char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
const DateTime hoot (2011, 7, 16, 0, 0, 0);
// this DataTime library doesn't work with years before 2000, so set it to 2000 and add the extra days manually
// Days from 1/1/1983 to 1/1/2000 is 6210
const DateTime katieBday (2000, 8, 17, 0, 0, 0);
//DateTime colinBday (2000, 5, 3, 0, 0, 0);
const int extraDays = 6210;
const int stdDelay = 4000;
const int stdFlash = stdDelay / 4;

// Define global (non-constants)
DateTime now;
// Defines the heading-countdown combination of text shown on the screen
// values are positive ints: 1, 2, 3, etc., button press increments the mode
// loop does nothing with mode 0, so after setup, button press will increment to 1, and begin the interface
byte displayMode = 0;
bool buttonDown = false;


//unsigned long startMillis;
//unsigned long currentMillis;

// Things that shouldn't have to be done every boot
void initializeKatieCountdown()
{
  // Set the RTC to the date & time this sketch was compiled
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  delay(1000);

  // DS1307 should be 0x68
  iic_discover();

  // Set SQW output frequency to 1 second
  writeRTC(0X07, B00010000);// 1Hz

  lcd.setCursor(0, 0);
  lcd.print("Initialization");
  lcd.setCursor(0, 1);
  lcd.print("complete");
  delay(stdDelay);
}

void iic_discover()
{
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
  delay(stdDelay); // wait 5 seconds for next scan
}

void writeRTC(byte location, byte data)
// writes data to location
{
  Wire.beginTransmission(0x68);
  Wire.write(location);
  Wire.write(data);
  Wire.endTransmission();
}

void displayDateTimeObject(DateTime dt) {

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(dt.year(), DEC);
  lcd.print('/');
  if (dt.month() < 10) {
    lcd.print('0');
  }
  lcd.print(dt.month(), DEC);
  lcd.print('/');
  if (dt.day() < 10) {
    lcd.print('0');
  }
  lcd.print(dt.day(), DEC);
  lcd.setCursor(0, 1);
  if (dt.hour() < 10) {
    lcd.print('0');
  }
  lcd.print(dt.hour(), DEC);
  lcd.print(':');
  if (dt.minute() < 10) {
    lcd.print('0');
  }
  lcd.print(dt.minute(), DEC);
  lcd.print(':');
  if (dt.second() < 10) {
    lcd.print('0');
  }
  lcd.print(dt.second(), DEC);

}

void displayTimeSpanObject(TimeSpan ts) {

  lcd.setCursor(0, 0);
  lcd.print(ts.days(), DEC);
  lcd.print(" days ");
  if (ts.hours() < 10) {
    lcd.print('0');
  }
  lcd.print(ts.hours(), DEC);
  lcd.print(" hrs ");
  lcd.setCursor(0, 1);
  if (ts.minutes() < 10) {
    lcd.print('0');
  }
  lcd.print(ts.minutes(), DEC);
  lcd.print(" mins ");
  if (ts.seconds() < 10) {
    lcd.print('0');
  }
  lcd.print(ts.seconds(), DEC);
  lcd.print(" secs   ");

}

void displayFlashingQuestionMark(byte pos, byte line) {

  for (byte num = 0; num < 5; num++) {
    lcd.setCursor(pos, line);
    lcd.print(' ');
    delay(stdFlash);
    lcd.setCursor(pos, line);
    lcd.print('?');
    delay(stdFlash);
  }

}

DateTime convertToNextDate(DateTime dt) {

  DateTime thisYear (now.year(), dt.month(), dt.day());

  if (now.unixtime() > thisYear.unixtime()) {
    // Date hass already passed, so add a year
    DateTime nextYear (now.year() + 1, dt.month(), dt.day());
    return nextYear;
  } else {
    return thisYear;
  }

}

void displayTimeTokBdayCounts() {

  DateTime nextKatieBday = convertToNextDate(katieBday);
  TimeSpan untilNextKatieBday = nextKatieBday - now;
  displayTimeSpanObject(untilNextKatieBday);

}

void displayTime() {

  lcd.setCursor(0, 0);
  lcd.print("How am I doing?");
  lcd.setCursor(0, 1);
  if (now.hour() < 10) {
    lcd.print('0');
  }
  lcd.print(now.hour(), DEC);
  lcd.print(':');
  if (now.minute() < 10) {
    lcd.print('0');
  }
  lcd.print(now.minute(), DEC);
  lcd.print(':');
  if (now.second() < 10) {
    lcd.print('0');
  }
  lcd.print(now.second(), DEC);

}

void displayTimeAliveCounts() {

  // extraDays is the number of days between actual Katie Birthday (1983) and katieBday (2000)
  TimeSpan alive = now - katieBday + TimeSpan(extraDays, 0, 0, 0);
  displayTimeSpanObject(alive);

}

void displayTimeToAnniversaryCounts() {

  DateTime nextAnniversary = convertToNextDate(hoot);
  TimeSpan untilNextAnniversary = nextAnniversary - now;
  displayTimeSpanObject(untilNextAnniversary);

}

void displayTimeTogetherCounts() {

  TimeSpan together = now - hoot;
  displayTimeSpanObject(together);

}

void buttonPress() {

  if (displayMode == numDisplayModes) {
    displayMode = 1;
  } else {
    displayMode++;
  }

  lcd.clear();
  Serial.print(F("displayMode is "));
  Serial.println(displayMode);
  switch (displayMode) {
    case 1:
      // How long has it been setup
      lcd.setCursor(0, 0);
      lcd.print("How long has it");
      lcd.setCursor(0, 1);
      lcd.print("been? ");
      lcd.print(hoot.year(), DEC);
      lcd.print('/');
      lcd.print(hoot.month(), DEC);
      lcd.print('/');
      lcd.print(hoot.day(), DEC);
      displayFlashingQuestionMark(4, 1);
      lcd.clear();
      break;
    case 2:
      // How long until next anniversary
      lcd.setCursor(0, 0);
      lcd.print("How long until");
      lcd.setCursor(0, 1);
      lcd.print("our anniversary?");
      displayFlashingQuestionMark(15, 1);
      lcd.clear();
      break;
    case 3:
      // Pin instructions for how long alive
      lcd.setCursor(0, 0);
      lcd.print("How long alive?");
      lcd.setCursor(0, 1);
      lcd.print("Connect GND<->A5");
      // don't clear screen here
      break;
    case 4:
      // Show the time
      lcd.setCursor(0, 0);
      lcd.print("Usually on time.");
      lcd.setCursor(0, 1);
      lcd.print("Some of the time");
      delay(stdDelay);
      lcd.clear();
      break;
    case 5:
      // How long until next Katie birthday
      lcd.setCursor(0, 0);
      lcd.print("Time we wait for");
      lcd.setCursor(0, 1);
      lcd.print("KB's birthdate?");
      displayFlashingQuestionMark(14, 1);
      lcd.clear();
      break;
    default:
      lcd.setCursor(0, 0);
      lcd.print("Bug displayMode:");
      lcd.setCursor(0, 1);
      lcd.print(displayMode);
      break;
  }

}

void setup() {

  Serial.begin(115200);
  Serial.println(F("Serial started"));

  if (! touch.begin()) {
    Serial.println("STMPE not found!");
    while (1);
  }
  Serial.println("Touch screen started");

  ImageReturnCode stat; // Status from image-reading functions

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  if (! rtc.begin()) {
    lcd.setCursor(0, 0);
    lcd.print("Couldn't find RTC");
    while (1);
  }

  //  initializeKatieCountdown();

  // welcome
  lcd.setCursor(0, 0);
  lcd.print("To Katie from CG");
  lcd.setCursor(0, 1);
  lcd.print("Heading for 2020");
  //  delay(stdDelay); The TFT setup is plenty of delay
  //  lcd.clear();



  tft.begin();          // Initialize screen
  tft.setRotation(3);    // Set rotation

  // The Adafruit_ImageReader constructor call (above, before setup())
  // accepts an uninitialized SdFat or FatFileSystem object. This MUST
  // BE INITIALIZED before using any of the image reader functions!
  Serial.print(F("Initializing SD filesystem..."));

  if (!sdfs.begin(SD_CS, SD_SCK_MHZ(25))) { // ESP32 requires 25 MHz limit
    Serial.println(F("SD begin() failed"));
    for (;;); // Fatal error, do not continue
  }

  Serial.println(F("OK!"));

  // Fill screen blue. Not a required step, this just shows that we're
  // successfully communicating with the screen.
  tft.fillScreen(HX8357_BLUE);
  //  tft.fillScreen(0);

  // Load full-screen BMP file 'cgkbresized.bmp' at position (0,0) (top left).
  // Notice the 'reader' object performs this, with 'tft' as an argument.
  Serial.print(F("Loading cgkbresized.bmp to screen..."));
  stat = reader.drawBMP("/cgkbresized.bmp", tft, 0, 0);
  // (Absolute path isn't necessary on most devices, but something
  // with the ESP32 SD library seems to require it.)
  reader.printStatus(stat);   // How'd we do?


  // initial hint
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Corner button...");
  lcd.setCursor(0, 1);
  lcd.print("New mode summons");
  // button press prompts next message


}

void loop() {

  // touch test
  uint16_t x, y;
  uint8_t z;
  if (touch.touched()) {
    // read x & y & z;
    while (! touch.bufferEmpty()) {
      Serial.print(touch.bufferSize());
      touch.readData(&x, &y, &z);
      Serial.print("->(");
      Serial.print(x); Serial.print(", ");
      Serial.print(y); Serial.print(", ");
      Serial.print(z);
      Serial.println(")");
    }
    touch.writeRegister8(STMPE_INT_STA, 0xFF); // reset all ints, in this example unneeded depending in use
  }
  delay(10);


  //  Check to see if button is down
  // A6 and A7 are analog inputs only, cannot use digitalRead() here
  // board has 10k pullup to side of button wired to A7, other side wired to ground
  // button press pulls A7 down to ground (typically 0, 1, or 2 analogRead value)
  if (analogRead(A7) < 10) {
    // don't react to a button that is still being held down from previous press
    if (!buttonDown) {
      buttonDown = true;
      buttonPress();
    }
  } else {
    // button not pressed, accept button input next loop
    buttonDown = false;
  }

  // Clear screen before arriving in this loop
  // Doing it here at beginning of loop causes flashing

  // update time on every loop
  // technically unnecessary, and waste of time, for displayMode == 0
  now = rtc.now();

  switch (displayMode) {
    case 0:
      // initial startup, do nothing until button pressed, and don't delay
      return;
    case 1:
      displayTimeTogetherCounts();
      break;
    case 2:
      displayTimeToAnniversaryCounts();
      break;
    case 3:
      if (digitalRead(A5) == LOW) {
        Serial.println("A5 low");
        displayTimeAliveCounts();
      } else {
        // don't wait for normal delay
        return;
      }
    case 4:
      displayTime();
      break;
    case 5:
      displayTimeTokBdayCounts();
      break;
  }

}
