/*
  Colin 2020-03-07
  Basic validation of Feather M4 with RTC + SD Featherwing
  This sketch combines the following examples into one:

  Adafruit
  RTCLib PCF8523 and DS1307SqwPin
  CardInfo
  BlinkWithoutDelay

  Test Date and time functions using a PCF8523 RTC connected via I2C and Wire lib
  Perform SD card test

  The circuit:
    SD card attached to SPI bus as follows:
 ** MOSI - pin 11 on Arduino Uno/Duemilanove/Diecimila
 ** MISO - pin 12 on Arduino Uno/Duemilanove/Diecimila
 ** CLK - pin 13 on Arduino Uno/Duemilanove/Diecimila
 ** CS - depends on your SD card shield or module.

  PCF8523 square wave to digital pin 5

*/


// include general RTC library that works with PCF8523
#include "RTClib.h"

// include the SD library:
#include <SPI.h>
#include <SD.h>

#define MONITOR_PIN 5


// Set up RTC variables
RTC_PCF8523 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;

// change this to match your SD shield or module;
// Arduino Ethernet shield: pin 4
// Adafruit SD shields and modules: pin 10
// Sparkfun SD shield: pin 8
// MKRZero SD: SDCARD_SS_PIN
const int chipSelect = 10;

// interrupt counter
int counter = 0;

// seconds between action
const int betweenSamples = 30;

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change:
const long interval = 1000;           // interval at which to blink (milliseconds)

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(MONITOR_PIN, INPUT_PULLUP);

  Serial.println("Starting RTC...");
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (! rtc.initialized()) {
    Serial.println("RTC is NOT running!");
  }
  // following line sets the RTC to the date & time this sketch was compiled
  Serial.println("Setting current time...");
   rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));


  Serial.print("\nSetting Square Wave...");
  rtc.writeSqwPinMode(PCF8523_SquareWave1HZ);
  Pcf8523SqwPinMode mode = rtc.readSqwPinMode();

  switch(mode) {
    case PCF8523_OFF:             Serial.println("OFF");       break;
    case PCF8523_SquareWave1HZ:   Serial.println("1Hz");       break;
    case PCF8523_SquareWave32HZ:  Serial.println("32Hz");      break;
    case PCF8523_SquareWave1kHz:  Serial.println("1kHz");      break;
    case PCF8523_SquareWave4kHz:  Serial.println("4.096kHz");  break;
    case PCF8523_SquareWave8kHz:  Serial.println("8.192kHz");  break;
    case PCF8523_SquareWave16kHz: Serial.println("16.384kHz"); break;
    case PCF8523_SquareWave32kHz: Serial.println("32.768kHz"); break;
    default:                      Serial.println("UNKNOWN");   break;
  }

  for (byte i=0; i < 5; i++) {
    showtime();
    delay(1000);
  }

  Serial.print("\nInitializing SD card...");

  // we'll use the initialization code from the utility libraries
  // since we're just testing if the card is working!
  if (!card.init(SPI_HALF_SPEED, chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("* is a card inserted?");
    Serial.println("* is your wiring correct?");
    Serial.println("* did you change the chipSelect pin to match your shield or module?");
    while (1);
  } else {
    Serial.println("Wiring is correct and a card is present.");
  }

  // print the type of card
  Serial.println();
  Serial.print("Card type:         ");
  switch (card.type()) {
    case SD_CARD_TYPE_SD1:
      Serial.println("SD1");
      break;
    case SD_CARD_TYPE_SD2:
      Serial.println("SD2");
      break;
    case SD_CARD_TYPE_SDHC:
      Serial.println("SDHC");
      break;
    default:
      Serial.println("Unknown");
  }

  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  if (!volume.init(card)) {
    Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
    while (1);
  }

  Serial.print("Clusters:          ");
  Serial.println(volume.clusterCount());
  Serial.print("Blocks x Cluster:  ");
  Serial.println(volume.blocksPerCluster());

  Serial.print("Total Blocks:      ");
  Serial.println(volume.blocksPerCluster() * volume.clusterCount());
  Serial.println();

  // print the type and size of the first FAT-type volume
  uint32_t volumesize;
  Serial.print("Volume type is:    FAT");
  Serial.println(volume.fatType(), DEC);

  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
  volumesize /= 2;                           // SD card blocks are always 512 bytes (2 blocks are 1KB)
  Serial.print("Volume size (Kb):  ");
  Serial.println(volumesize);
  Serial.print("Volume size (Mb):  ");
  volumesize /= 1024;
  Serial.println(volumesize);
  Serial.print("Volume size (Gb):  ");
  Serial.println((float)volumesize / 1024.0);

  Serial.println("\nFiles found on the card (name, date and size in bytes): ");
  root.openRoot(volume);

  // list all files in the card with date and size
  root.ls(LS_R | LS_DATE | LS_SIZE);

  // We're done with setup stuff, now start the interrupt loop
  attachInterrupt(MONITOR_PIN, onAlarm, FALLING);


}

void showtime() {
    DateTime now = rtc.now();

    Serial.print("The current time is: ");
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();

    Serial.print(" since midnight 1/1/1970 = ");
    Serial.print(now.unixtime());
    Serial.print("s = ");
    Serial.print(now.unixtime() / 86400L);
    Serial.println("d");

    // calculate a date which is 7 days, 12 hours and 30 seconds into the future
    DateTime future (now + TimeSpan(7,12,30,6));

    Serial.print(" now + 7d + 12h + 30m + 6s: ");
    Serial.print(future.year(), DEC);
    Serial.print('/');
    Serial.print(future.month(), DEC);
    Serial.print('/');
    Serial.print(future.day(), DEC);
    Serial.print(' ');
    Serial.print(future.hour(), DEC);
    Serial.print(':');
    Serial.print(future.minute(), DEC);
    Serial.print(':');
    Serial.print(future.second(), DEC);
    Serial.println();
    Serial.println();
}

void onAlarm() {
  noInterrupts();
  ++counter;
  if (counter == betweenSamples) {
    unsigned long currentMillis = millis();
    Serial.print("\nElapsed millis(): ");
    Serial.println(currentMillis - previousMillis);
    showtime();
    previousMillis = currentMillis;
//    Serial.println(millis());
    counter = 0;
  }
  Serial.println(counter);
  interrupts();
}

void loop(void) {
}
