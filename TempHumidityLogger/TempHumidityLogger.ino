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


#include <RTClib.h>                     // PCF8523
#include <SPI.h>                        // SD
#include <SdFat.h>                      // Filesystem
#include <Adafruit_SHT31.h>             // Temperature and Humidity
#include <Adafruit_BMP280.h>            // Barometric Pressure, Temperature, Altitude (note, relies on Adafruit Unified Sensor library)

// Log file base name.  Must be six characters or less.
#define FILE_BASE_NAME "Vent1_"

// seconds between collecting samples
#define SECONDSBETWEENSAMPLES 30

#define TIMERINTERRUPTPIN 5        // PCF8523
#define SD_CS_PIN 10
#define error(msg) sd.errorHalt(F(msg))

// voltage reading ADC settings
// https://learn.adafruit.com/bluefruit-nrf52-feather-learning-guide/nrf52-adc
#define VREFMULTIPLIER AR_INTERNAL_2_4  // 0..2.4v at divider, so 4.8v capable
#define ADCBITS 10                      // fast reads, plenty of precision
#define ADCDIVISIONS 1024               // 2^10 = 1024

#define DEBUG
#define USESERIAL1 // Plug in FTDI to GND and Tx to read status
#define HASBMP // Adafruit bmp280 barometric pressure and temperature sensor, not on Feather M4 express

#ifdef USESERIAL1
#define DEBUGSERIALPORT Serial1
#else
#define DEBUGSERIALPORT Serial
#endif

#define SETDATETIME // Set the RTC time to system compile time, configure 1 Hz Square Wave. Runs about 12 seconds behind.
#define TIMESINCECOMPILE 12 // Adjust for the lag between setting __TIME__ during compile, and actually setting time on the RTC

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
  DEBUGSERIALPORT.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  DEBUGSERIALPORT.print("\nStarting RTC...");
  if (! rtc.begin()) {
    DEBUGSERIALPORT.println("Couldn't find RTC");
    while (1);
  }
  DEBUGSERIALPORT.println("Done.");

  if (! rtc.initialized()) {
    DEBUGSERIALPORT.println("RTC is NOT running!");
  }

#ifdef SETDATETIME
  DEBUGSERIALPORT.print("\nSetting current time...");
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)).unixtime() + TIMESINCECOMPILE);
  DEBUGSERIALPORT.println("Done.");
#endif

  DEBUGSERIALPORT.print("\nInitializing SHT31...");
  if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
    DEBUGSERIALPORT.println("Couldn't find SHT31");
    while (1) delay(1);
  }
  DEBUGSERIALPORT.println("Done.");

#ifdef HASBMP
  DEBUGSERIALPORT.print("\nInitializing BMP280...");
  if (!bmp280.begin()) {
    DEBUGSERIALPORT.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1) delay(1);;
  }
  // From Adafruit example, where they say 'Default settings from datasheet'
  bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,      /* Operating Mode */
                     Adafruit_BMP280::SAMPLING_X1,      /* Temp. oversampling */
                     Adafruit_BMP280::SAMPLING_X2,      /* Pressure oversampling */
                     Adafruit_BMP280::FILTER_OFF,       /* Filtering */
                     Adafruit_BMP280::STANDBY_MS_1000); /* Standby time (inconsequential in FORCED mode) */
  DEBUGSERIALPORT.println("Done.");
#endif

  DEBUGSERIALPORT.print("\nInitializing SD card...");
  // Initialize at the highest speed supported by the board that is
  // not over 50 MHz. Try a lower speed if SPI errors occur.
  if (!sd.begin(SD_CS_PIN, SPI_HALF_SPEED)) {
    DEBUGSERIALPORT.println(F("SD card failed to initialize"));
    sd.initErrorHalt();
  }
  DEBUGSERIALPORT.println("Done.");

  DEBUGSERIALPORT.print("\nOpening data file...");
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
  DEBUGSERIALPORT.println("Done.");

  DEBUGSERIALPORT.print(F("Logging to: "));
  DEBUGSERIALPORT.println(fileName);

  // We're done with setup stuff, now start the interrupt loop.
  enableRTC();

}

void enableRTC() {
  // Take the first sample at beginning of the minute
  DEBUGSERIALPORT.println(F("\nConfiguring countdown to take first sample at beginning of minute..."));
  byte currSec;
  DateTime now;
  do {
    now = rtc.now();
    currSec = now.second();
#ifdef DEBUG
    DEBUGSERIALPORT.print(F("Current second: "));
    DEBUGSERIALPORT.println(currSec);
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
  DEBUGSERIALPORT.println("\nCountdown timer enabled.");
  showTime(now);
}

void disableRTC() {
  rtc.disableCountdownTimer();
  detachInterrupt(TIMERINTERRUPTPIN);
}

void printStoredSamples() {
  file.rewind();
  while (file.available()) {
    DEBUGSERIALPORT.write(file.read());
  }
}

void showTime(DateTime now) {
  DEBUGSERIALPORT.print("The current time is: ");
  DEBUGSERIALPORT.print(now.year(), DEC);
  DEBUGSERIALPORT.print('/');
  DEBUGSERIALPORT.print(now.month(), DEC);
  DEBUGSERIALPORT.print('/');
  DEBUGSERIALPORT.print(now.day(), DEC);
  DEBUGSERIALPORT.print(" ");
  DEBUGSERIALPORT.print(now.hour(), DEC);
  DEBUGSERIALPORT.print(':');
  DEBUGSERIALPORT.print(now.minute(), DEC);
  DEBUGSERIALPORT.print(':');
  DEBUGSERIALPORT.print(now.second(), DEC);
  DEBUGSERIALPORT.print(" ");
  DEBUGSERIALPORT.println(now.unixtime());
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

#ifdef HASBMP
  float tmp_bmp280 = bmp280.readTemperature();
  float bar_bmp280 = bmp280.readPressure();
  float alt_bmp280 = bmp280.readAltitude(1013.25);
#endif

  float voltage = getVoltage();

  // Write data to file. Start with unix time.
  file.print(now.unixtime());
  file.print(',');
  file.print(voltage);
  file.print(',');
  file.print(relh_sht31);
  file.print(',');
  file.print(temp_sht31);
#ifdef HASBMP
  file.print(',');
  file.print(tmp_bmp280);
  file.print(',');
  file.print(bar_bmp280);
  file.print(',');
  file.print(alt_bmp280);
#endif
  file.println(); // End with a new line


#ifdef DEBUG
  // Pretty print individual labeled measures
  DEBUGSERIALPORT.println();
  showTime(now);
  DEBUGSERIALPORT.print("Battery     V = "); DEBUGSERIALPORT.println(voltage);
  DEBUGSERIALPORT.print("SHT31  Hum  % = "); DEBUGSERIALPORT.println(relh_sht31);
  DEBUGSERIALPORT.print("SHT31  Tmp *C = "); DEBUGSERIALPORT.println(temp_sht31);
#ifdef HASBMP
  DEBUGSERIALPORT.print("BMP280 Tmp *C = "); DEBUGSERIALPORT.println(tmp_bmp280);
  DEBUGSERIALPORT.print("BMP280 Bar Pa = "); DEBUGSERIALPORT.println(bar_bmp280);
  DEBUGSERIALPORT.print("BMP280 Alt  m = "); DEBUGSERIALPORT.println(alt_bmp280);
#endif
#endif

  // Write data string to Serial. Start with unix time.
  DEBUGSERIALPORT.println();
  DEBUGSERIALPORT.print(now.unixtime());
  DEBUGSERIALPORT.print(',');
  DEBUGSERIALPORT.print(voltage);
  DEBUGSERIALPORT.print(',');
  DEBUGSERIALPORT.print(relh_sht31);
  DEBUGSERIALPORT.print(',');
  DEBUGSERIALPORT.print(temp_sht31);
#ifdef HASBMP
  DEBUGSERIALPORT.print(',');
  DEBUGSERIALPORT.print(tmp_bmp280);
  DEBUGSERIALPORT.print(',');
  DEBUGSERIALPORT.print(bar_bmp280);
  DEBUGSERIALPORT.print(',');
  DEBUGSERIALPORT.print(alt_bmp280);
#endif

  // End with a new line
  DEBUGSERIALPORT.println();

  // Force data to SD and update the directory entry to avoid data loss.
  if (!file.sync() || file.getWriteError()) {
    error("write error");
  }

  // So that loop() will skip until it's time to take another
  timeForSample = false;
}

void countdownOver() {
  timeForSample = true;
}

void loop(void) {
  // Temp sensor uses I2C which doesn't work reliably during an interrupt,
  // so initiate sampling here, immediately after the last interrupt.
  if (timeForSample) {
    #ifdef DEBUG
    DEBUGSERIALPORT.println("Taking sample");
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
