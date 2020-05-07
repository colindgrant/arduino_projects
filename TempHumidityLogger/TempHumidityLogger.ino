/*
  Colin 2020-04-21
  Basic validation of Feather M4 with RTC + SD Featherwing
  Logs time, battery voltage, temp, humidity to CSV file on SD card

  This sketch combines the following examples into one:

  Adafruit
  RTCLib PCF8523 and DS1307SqwPin
  CardInfo
  BlinkWithoutDelay

*/

#include <Adafruit_BMP280.h> // Barometric Pressure, Temperature, Altitude (note, relies on Adafruit Unified Sensor library)
#include <Adafruit_SHT31.h>  // Temperature and Humidity
#include <RTClib.h>          // PCF8523
#include <SPI.h>             // SD
#include <SdFat.h>           // Filesystem


////////////////////////////////////////////////////////////////////////////////
// Dev selections for switching between board types, etc.
//
#define FILE_BASE_NAME "Vent1_" // log file, six characters or less
#define SECONDSBETWEENSAMPLES 30 // seconds between collecting samples
#define USESERIAL1 // Plug in FTDI to GND and Tx to read status
//#define SETDATETIME // Set the RTC time to system compile time
//#define DEBUG
//
////////////////////////////////////////////////////////////////////////////////


// Things that shouldn't have to be messed with
#define TIMERINTERRUPTPIN 5 // PCF8523 INT/SQW, common Feather input
#define SD_CS_PIN 10 // Feather standard
#define error(msg) sd.errorHalt(F(msg))

#ifdef ARDUINO_FEATHER_M4 // Has SHT31 backpack
#define BOARDNAME "Adafruit Feather M4 Express"
#define TIMESINCECOMPILE 8 // Lag between compile __TIME__, and setting RTC
#define VREFMULTIPLIER AR_INTERNAL2V4 // 0..2.4v at divider, so 4.8v capable
#define PIN_VBAT A6
#endif

#ifdef ARDUINO_NRF52840_FEATHER_SENSE
#define BOARDNAME "Adafruit Feather nRF52840 Sense"
#define TIMESINCECOMPILE 12 // Lag between compile __TIME__, and setting RTC
#define HASBMP // Adafruit bmp280 on Feather Sense not on Feather M4 express
#define VREFMULTIPLIER AR_INTERNAL_2_4 // 0..2.4v at divider, so 4.8v capable
#endif

#define ADCBITS 10 // fast reads, plenty of precision
#define ADCDIVISIONS 1024 // 2^10 = 1024

#ifdef USESERIAL1
#define MYSERIAL Serial1 // FTDI on Tx Rx pins
#else
#define MYSERIAL Serial  // Native USB
#endif

// Set up RTC
RTC_PCF8523 rtc;

// Set up Temp and Humidity sensor
Adafruit_SHT31 sht31;

#ifdef HASBMP
// Set up Baro, Temp, and Altitude sensor
Adafruit_BMP280 bmp280;
#endif

// File system object
SdFat sd;

// Log file
SdFile file;

const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
char fileName[13] = FILE_BASE_NAME "00.csv";

// whether it is time to take a new sample
bool timeForSample = false;

void setup() {

  // Set up ADC for voltage reading
  analogReadResolution(ADCBITS);
  analogReference(VREFMULTIPLIER);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TIMERINTERRUPTPIN, INPUT_PULLUP);

  // Open serial communications and wait for port to open:
  MYSERIAL.begin(115200);
  if (!Serial) delay(500); // for nrf52840 with native usb

  MYSERIAL.print("Board type: ");
  MYSERIAL.println(BOARDNAME);

  MYSERIAL.print(F("\nStarting RTC..."));
  if (!rtc.begin()) {
    MYSERIAL.println(F("Couldn't find RTC"));
    while (1);
  }
  MYSERIAL.println(F("Done."));

  if (!rtc.initialized()) {
    MYSERIAL.println(F("RTC is NOT initialized!"));
    while (1);
  }

#ifdef SETDATETIME
  MYSERIAL.print(F("\nSetting current time..."));
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)).unixtime() + TIMESINCECOMPILE);
  rtc.deconfigureAllTimers();
  MYSERIAL.println(F("Done."));
#endif

  MYSERIAL.print(F("\nInitializing SHT31..."));
  if (!sht31.begin(0x44)) { // Set to 0x45 for alternate i2c addr
    MYSERIAL.println(F("Couldn't find SHT31"));
    while (1);
  }
  MYSERIAL.println(F("Done."));

#ifdef HASBMP
  MYSERIAL.print(F("\nInitializing BMP280..."));
  if (!bmp280.begin()) {
    MYSERIAL.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
  // From Adafruit example, where they say 'Default settings from datasheet'
  bmp280.setSampling(
      Adafruit_BMP280::MODE_NORMAL,      /* Operating Mode */
      Adafruit_BMP280::SAMPLING_X1,      /* Temp. oversampling */
      Adafruit_BMP280::SAMPLING_X2,      /* Pressure oversampling */
      Adafruit_BMP280::FILTER_OFF,       /* Filtering */
      Adafruit_BMP280::STANDBY_MS_1000); /* Standby time (inconsequential in
                                            FORCED mode) */
  MYSERIAL.println(F("Done."));
#endif

  MYSERIAL.print(F("\nInitializing SD card..."));
  // Initialize at the highest speed supported by the board that is
  // not over 50 MHz. Try a lower speed if SPI errors occur.
  if (!sd.begin(SD_CS_PIN, SPI_HALF_SPEED)) {
    MYSERIAL.println(F("SD card failed to initialize"));
    sd.initErrorHalt();
  }
  MYSERIAL.println(F("Done."));

  MYSERIAL.print(F("\nOpening data file..."));
  // Find an unused file name.
  if (BASE_NAME_SIZE > 6) {
    error("FILE_BASE_NAME too long");
  }
  while (sd.exists(fileName)) {
    if (fileName[BASE_NAME_SIZE + 1] != '9') {
      fileName[BASE_NAME_SIZE + 1]++;
    } else if (fileName[BASE_NAME_SIZE] != '9') {
      fileName[BASE_NAME_SIZE + 1] = '0';
      fileName[BASE_NAME_SIZE]++;
    } else {
      error("Can't create file name");
    }
  }
  if (!file.open(fileName, O_RDWR | O_CREAT | O_EXCL)) {
    error("file.open");
  }
  MYSERIAL.println(F("Done."));

  MYSERIAL.print(F("Logging to: "));
  MYSERIAL.println(fileName);

  // We're done with setup stuff, now start the interrupt loop.
  enableRTC();
}

void enableRTC() {
  // Take the first sample at beginning of the minute
  MYSERIAL.println(F("\nConfiguring countdown to take first sample at "
                     "beginning of minute..."));
  byte currSec;
  DateTime now = rtc.now();
  showTime(now);
  do {
    now = rtc.now();
    currSec = now.second();
#ifdef DEBUG
    MYSERIAL.print(F("Current second: "));
    MYSERIAL.println(currSec);
#endif
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(490);
  } while (currSec != (59 - SECONDSBETWEENSAMPLES));

  // Enable and attach to countdown timer, each time countdown is over,
  // call countdownOver(), which will take a sample
  rtc.enableCountdownTimer(PCF8523_FrequencySecond, SECONDSBETWEENSAMPLES);
  attachInterrupt(TIMERINTERRUPTPIN, countdownOver, FALLING);
  MYSERIAL.println(F("\nCountdown timer enabled."));
  showTime(now);
}

void disableRTC() {
  rtc.disableCountdownTimer();
  detachInterrupt(TIMERINTERRUPTPIN);
}

void printStoredSamples() {
  file.rewind();
  while (file.available()) {
    MYSERIAL.write(file.read());
  }
}

void showTime(DateTime now) {
  MYSERIAL.print(F("The current time is: "));
  MYSERIAL.print(now.year(), DEC);
  MYSERIAL.print('/');
  MYSERIAL.print(now.month(), DEC);
  MYSERIAL.print('/');
  MYSERIAL.print(now.day(), DEC);
  MYSERIAL.print(' ');
  MYSERIAL.print(now.hour(), DEC);
  MYSERIAL.print(':');
  MYSERIAL.print(now.minute(), DEC);
  MYSERIAL.print(':');
  MYSERIAL.print(now.second(), DEC);
  MYSERIAL.print(' ');
  MYSERIAL.println(now.unixtime());
}

float getVoltage() {
  // Let the ADC settle
  delay(1);

  // 2^10 = 1024 divisions, PIN_VBAT is connected to even divider
  // Input max 2.1v for fully charged 4.2v battery, use 2.4 reference
  return (analogRead(PIN_VBAT) * 2.4 / ADCDIVISIONS * 2);
}

void takeSample() {
  // Here is where data collection and storage happens
  DateTime now = rtc.now();

  float temp_sht31 = sht31.readTemperature();
  float relh_sht31 = sht31.readHumidity();

  float tmp_bmp280 = 0;
  float bar_bmp280 = 0;
  float alt_bmp280 = 0;
#ifdef HASBMP
  tmp_bmp280 = bmp280.readTemperature();
  bar_bmp280 = bmp280.readPressure();
  alt_bmp280 = bmp280.readAltitude(1013.25);
#endif

  float voltage = getVoltage();

  // Char arrary for the CSV data string to be be stored on SD card
  // Should only need 50 chars, but altitude can be negative, etc.
  char csv[64];
  sprintf(csv, "%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", now.unixtime(), voltage,
          relh_sht31, temp_sht31, tmp_bmp280, bar_bmp280, alt_bmp280);

  // Write data to SD file, note string already ends with a new line
  file.print(csv);

#ifdef DEBUG
  // Pretty print individual labeled measures
  MYSERIAL.println();
  showTime(now);
  MYSERIAL.print(F("Battery     V = ")); MYSERIAL.println(voltage);
  MYSERIAL.print(F("SHT31  Hum  % = ")); MYSERIAL.println(relh_sht31);
  MYSERIAL.print(F("SHT31  Tmp *C = ")); MYSERIAL.println(temp_sht31);
  MYSERIAL.print(F("BMP280 Tmp *C = ")); MYSERIAL.println(tmp_bmp280);
  MYSERIAL.print(F("BMP280 Bar Pa = ")); MYSERIAL.println(bar_bmp280);
  MYSERIAL.print(F("BMP280 Alt  m = ")); MYSERIAL.println(alt_bmp280);
#endif

  // Write data string to Serial
  MYSERIAL.println(csv);

  // Force data to SD and update the directory entry to avoid data loss.
  if (!file.sync() || file.getWriteError()) {
    error("write error");
  }

  // So that loop() will skip until it's time to take another
  timeForSample = false;
}

void countdownOver() {
  // Time froze in here, do not try to do anything complex
  // Set flag here to act on in loop
  timeForSample = true;
}

void loop(void) {
  // Temp sensor uses I2C which doesn't work reliably during an interrupt,
  // so initiate sampling here, immediately after the last interrupt.
  if (timeForSample) {
#ifdef DEBUG
    MYSERIAL.println(F("Taking sample"));
#endif
    // Built-in LED on to indicate sampling
    digitalWrite(LED_BUILTIN, HIGH);
    takeSample();
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);

#ifdef DEBUG
    // don't do this forever, the time it takes to print will increase
    printStoredSamples();
#endif
  }
}
