#include <Wire.h>
#include <LiquidCrystal.h>
#include <RTClib.h>

#include <Adafruit_GFX.h>         // Core graphics library
#include <Adafruit_HX8357.h>      // Hardware-specific library
#include <SdFat.h>                // SD card & FAT filesystem library
#include <Adafruit_ImageReader.h> // Image-reading functions
#include <Adafruit_STMPE610.h>



// TFT/SD pins over and above default SPI pins for MOSI, MISO, CLK
#define TFT_CS   10
#define TFT_DC   A0
#define SD_CS    A3 // CCS on board

SdFat                SD;         // SD card filesystem
Adafruit_ImageReader reader(SD); // Image-reader object, pass in SD filesys

Adafruit_HX8357        tft    = Adafruit_HX8357(TFT_CS, TFT_DC);
int32_t                width  = 0, // BMP image dimensions (Adafruit ex used int32_t here)
                       height = 0;

LiquidCrystal lcd(9, 8, 6, 5, 4, 3);

RTC_DS1307 rtc;

// Option #1 - uses I2C, connect to hardware I2C port only!
// SCL to I2C clock (#A5 on Uno) and SDA to I2C data (#A4 on Uno)
// tie MODE to GND and POWER CYCLE (there is no reset pin)
Adafruit_STMPE610 touch = Adafruit_STMPE610();

const byte numDisplayModes = 5;

// Defines the heading-countdown combination of text shown on the screen
// values are positive ints: 1, 2, 3, etc., button press increments the mode
// loop does nothing with mode 0, so after setup, button press will increment to 1, and begin the interface
byte displayMode = 0;

bool buttonDown = false;

//char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

DateTime hoot (2011, 7, 16, 0, 0, 0);

// this DataTime library doesn't work with years before 2000, so set it to 2000 and add the extra days manually
// Days from 1/1/1983 to 1/1/2000 is 6210
DateTime katieBday (2000, 8, 17, 0, 0, 0);
//DateTime colinBday (2000, 5, 3, 0, 0, 0);
int extraDays = 6210;

DateTime now;

int stdDelay = 4000;
int stdFlash = stdDelay / 4;

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

void displayDateTimeObject(DateTime dt){

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(dt.year(), DEC);
  lcd.print('/');
  lcd.print(dt.month(), DEC);
  lcd.print('/');
  lcd.print(dt.day(), DEC);
  lcd.setCursor(0, 1);
  lcd.print(dt.hour(), DEC);
  lcd.print(':');
  lcd.print(dt.minute(), DEC);
  lcd.print(':');
  lcd.print(dt.second(), DEC);
  
}

DateTime convertToNextDate(DateTime dt){

  DateTime thisYear (now.year(), dt.month(), dt.day());

  if (now.unixtime() > thisYear.unixtime()) {
    // Date hass already passed, so add a year
    DateTime nextYear (now.year() + 1, dt.month(), dt.day());
    return nextYear;
  } else {
    return thisYear;
  }
  
}

void displayTimeTokBdayCounts()
{

  DateTime nextKatieBday = convertToNextDate(katieBday);
  TimeSpan untilNextKatieBday = nextKatieBday - now;

  lcd.setCursor(0, 0);
  lcd.print(untilNextKatieBday.days(), DEC);
  lcd.print(" days ");
  lcd.print(untilNextKatieBday.hours(), DEC);
  lcd.print(" hrs ");
  lcd.setCursor(0, 1);
  lcd.print(untilNextKatieBday.minutes(), DEC);
  lcd.print(" mins ");
  lcd.print(untilNextKatieBday.seconds(), DEC);
  lcd.print(" secs   ");
  
}

void displayTime()
{

  lcd.setCursor(0, 0);
  lcd.print("How am I doing?");
  lcd.setCursor(0, 1);
  lcd.print(now.hour(), DEC);
  lcd.print(':');
  lcd.print(now.minute(), DEC);
  lcd.print(':');
  lcd.print(now.second(), DEC);

}

void displayTimeAliveCounts()
{

  // extraDays is the number of days between actual Katie Birthday (1983) and katieBday (2000)
  TimeSpan alive = now - katieBday + TimeSpan(extraDays, 0, 0, 0);

  lcd.setCursor(0, 0);
  lcd.print(alive.days(), DEC);
  lcd.print(" days ");
  lcd.print(alive.hours(), DEC);
  lcd.print(" hrs ");
  lcd.setCursor(0, 1);
  lcd.print(alive.minutes(), DEC);
  lcd.print(" mins ");
  lcd.print(alive.seconds(), DEC);
  lcd.print(" secs   ");
  
}

void displayTimeToAnniversaryCounts()
{

  DateTime nextAnniversary = convertToNextDate(hoot);
  TimeSpan untilNextAnniversary = nextAnniversary - now;

  lcd.setCursor(0, 0);
  lcd.print(untilNextAnniversary.days(), DEC);
  lcd.print(" days ");
  lcd.print(untilNextAnniversary.hours(), DEC);
  lcd.print(" hrs ");
  lcd.setCursor(0, 1);
  lcd.print(untilNextAnniversary.minutes(), DEC);
  lcd.print(" mins ");
  lcd.print(untilNextAnniversary.seconds(), DEC);
  lcd.print(" secs   ");
  
}

void displayTimeTogetherCounts()
{

  TimeSpan together = now - hoot;

  lcd.setCursor(0, 0);
  lcd.print(together.days(), DEC);
  lcd.print(" days ");
  lcd.print(together.hours(), DEC);
  lcd.print(" hrs ");
  lcd.setCursor(0, 1);
  lcd.print(together.minutes(), DEC);
  lcd.print(" mins ");
  lcd.print(together.seconds(), DEC);
  lcd.print(" secs   ");
  
}

void buttonPress()
{

  if (displayMode == numDisplayModes)
  {
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
    
      for (byte num = 0; num < 5; num++) {
        lcd.setCursor(4, 1);
        lcd.print(' ');
        delay(stdFlash);
        lcd.setCursor(4, 1);
        lcd.print('?');
        delay(stdFlash);
      }
      lcd.clear();
      break;
    case 2:
      // How long until next anniversary
      lcd.setCursor(0, 0);
      lcd.print("How long until");
      lcd.setCursor(0, 1);
      lcd.print("our anniversary?");
  
      for (byte num = 0; num < 5; num++) {
        lcd.setCursor(15, 1);
        lcd.print(' ');
        delay(stdFlash);
        lcd.setCursor(15, 1);
        lcd.print('?');
        delay(stdFlash);
      }
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
  
      for (byte num = 0; num < 5; num++) {
        lcd.setCursor(14, 1);
        lcd.print(' ');
        delay(stdFlash);
        lcd.setCursor(14, 1);
        lcd.print('?');
        delay(stdFlash);
      }
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
    while(1);
  }
  Serial.println("Touch screen started");

//  startMillis = millis();

  ImageReturnCode stat; // Status from image-reading functions
  
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  if (! rtc.begin()) {
    lcd.setCursor(0, 0);
    lcd.print("Couldn't find RTC");
    while (1);
  }

//  initializeKatieCountdown();

//  pinMode(LED_BUILTIN, OUTPUT);  // sets the digital pin 13 as output
  pinMode(A5, INPUT_PULLUP);  // set pull-up on analog pin 5

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

  if(!SD.begin(SD_CS, SD_SCK_MHZ(25))) { // ESP32 requires 25 MHz limit
    Serial.println(F("SD begin() failed"));
    for(;;); // Fatal error, do not continue
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

//  // Query the dimensions of image 'parrot.bmp' WITHOUT loading to screen:
//  Serial.print(F("Querying cgkbresized.bmp image size..."));
//  stat = reader.bmpDimensions("/cgkbresized.bmp", &width, &height);
//  reader.printStatus(stat);   // How'd we do?
//  if(stat == IMAGE_SUCCESS) { // If it worked, print image size...
//    Serial.print(F("Image dimensions: "));
//    Serial.print(width);
//    Serial.write('x');
//    Serial.println(height);
//  }


  // Merry Christmas!
  
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
  // for touch example to work, loop needs to repeat quickly, not wait 1 second as it is currenlty configured
  return;

  // A6 and A7 are analog inputs only, cannot use digitalRead here
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

//  for(byte r=0; r<4; r++) { // For each of 4 rotations...
//    tft.setRotation(r);    // Set rotation
//    tft.fillScreen(0);     // and clear screen
//
//    // Load 4 copies of the 'parrot.bmp' image to the screen, some
//    // partially off screen edges to demonstrate clipping. Globals
//    // 'width' and 'height' were set by bmpDimensions() call in setup().
//    for(byte i=0; i<4; i++) {
//      reader.drawBMP("/cgkbresized.bmp", tft,
//        (tft.width()  * i / 3) - (width  / 2),
//        (tft.height() * i / 3) - (height / 2));
//    }
//
//    delay(1000); // Pause 1 sec.
//
//    // Draw 50 Welsh dragon flags in random positions. This has no effect
//    // on memory-constrained boards like the Arduino Uno, where the image
//    // failed to load due to insufficient RAM, but it's NOT fatal.
////    for(byte i=0; i<50; i++) {
////      // Rather than reader.drawBMP() (which works from SD card),
////      // a different function is used for RAM-resident images:
////      img.draw(tft,                                    // Pass in tft object
////        (int16_t)random(-img.width() , tft.width()) ,  // Horiz pos.
////        (int16_t)random(-img.height(), tft.height())); // Vert pos
////      // Reiterating a prior point: img.draw() does nothing and returns
////      // if the image failed to load. It's unfortunate but not disastrous.
////    }
//
//    delay(1000); // Pause 2 sec.
//  }
//  currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
//  Serial.println(currentMillis - startMillis);  //the period which has elapsed
//  startMillis = currentMillis;  //IMPORTANT to save the time
 
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

  // use millis() to find out exactly how long the loop cycle time is, try to sync to LED 1 Hz flash
  delay (988);
  
}
