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
#include <Adafruit_SHT31.h>             // Temp and humidity

// Log file base name.  Must be six characters or less.
#define FILE_BASE_NAME "Vent1_"

#define SQUARE_WAVE_SIGNAL_PIN 5        // PCF8523
#define SD_CS_PIN 10
#define error(msg) sd.errorHalt(F(msg))

// voltage reading ADC settings
// https://learn.adafruit.com/bluefruit-nrf52-feather-learning-guide/nrf52-adc
#define VREFMULTIPLIER AR_INTERNAL_2_4  // 0..2.4v
#define ADCBITS 10                      // fast reads
#define ADCDIVISIONS 1024               // 2^10 = 1024

// Set up RTC
RTC_PCF8523 rtc;

// Set up Temp and Humidity sensor
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// File system object
SdFat sd;

// Log file
SdFile file;

const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
char fileName[13] = FILE_BASE_NAME "00.csv";

// seconds between action
const int betweenSamples = 5;

// interrupt counter
int counter = 0;

// whether it is time to take a new sample
bool timeForSample = false;


void setup() {

  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // Set up ADC for voltage reading
  analogReadResolution(ADCBITS);
  analogReference(VREFMULTIPLIER);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SQUARE_WAVE_SIGNAL_PIN, INPUT_PULLUP);

  Serial.print("\nStarting RTC...");
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  Serial.println("Done.");

  if (! rtc.initialized()) {
    Serial.println("RTC is NOT running!");
  }

  Serial.print("\nSetting current time...");
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  Serial.println("Done.");

  Serial.print("\nSetting 1Hz Square Wave...");
  rtc.writeSqwPinMode(PCF8523_SquareWave1HZ);
  Pcf8523SqwPinMode mode = rtc.readSqwPinMode();
  Serial.println("Done.");

  Serial.print("\nInitializing SHT31...");
  if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
    Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  }
  Serial.println("Done.");

  Serial.print("\nInitializing SD card...");
  // Initialize at the highest speed supported by the board that is
  // not over 50 MHz. Try a lower speed if SPI errors occur.
  if (!sd.begin(SD_CS_PIN, SPI_HALF_SPEED)) {
    Serial.println(F("SD card failed to initialize"));
    sd.initErrorHalt();
  }
  Serial.println("Done.");

  Serial.print("\nOpening data file...");
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
  Serial.println("Done.");

  Serial.print(F("Logging to: "));
  Serial.println(fileName);

  // We're done with setup stuff, now start the interrupt loop.
  enableRTC();

}

void enableRTC() {
  // Square wave input has pullup, when RTC pulls from high to low,
  // call onAlarm(), which will determine if it's time to take a sample.
  // TODO: make this based on an actual alarm signal (>1s) from RTC.
  attachInterrupt(SQUARE_WAVE_SIGNAL_PIN, onAlarm, FALLING);
}

void disableRTC() {
  detachInterrupt(SQUARE_WAVE_SIGNAL_PIN);
}

void printStoredSamples() {
  file.rewind();
  while (file.available()) {
    Serial.write(file.read());
  }
}

void showTime(DateTime now) {
  Serial.print("The current time is: ");
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.print(" ");
  Serial.println(now.unixtime());
}

float getVoltage() {
  // 2^10 = 1024 divisions, PIN_VBAT is connected to even divider
  // Input max 2.1v for fully charged 4.2v battery, use 2.4 reference
  return (analogRead(PIN_VBAT) * 2.4 / ADCDIVISIONS * 2);
}

void takeSample() {
  // Here is where data collection and storage happens
  float t = sht31.readTemperature();
  float h = sht31.readHumidity();
  DateTime now = rtc.now();
  float v = getVoltage();

  #ifdef DEBUG
  showTime(now);
  if (! isnan(t)) {  // check if 'is not a number'
    Serial.print("Temp *C = "); Serial.println(t);
  } else { 
    Serial.println("Failed to read temperature");
  }
  if (! isnan(h)) {  // check if 'is not a number'
    Serial.print("Hum. % = "); Serial.println(h);
  } else { 
    Serial.println("Failed to read humidity");
  }
  Serial.print("Voltage is: ");
  Serial.println(v);
  #endif

  // Write data to file. Start with unix time.
  file.print(now.unixtime());
  file.print(',');
  file.print(v);
  file.print(',');
  file.print(t);
  file.print(',');
  file.print(h);

  // End with a new line
  file.println();

  // Write data to file. Start with unix time.
  Serial.println();
  Serial.print(now.unixtime());
  Serial.print(',');
  Serial.print(v);
  Serial.print(',');
  Serial.print(t);
  Serial.print(',');
  Serial.print(h);

  // End with a new line
  Serial.println();

  // Force data to SD and update the directory entry to avoid data loss.
  if (!file.sync() || file.getWriteError()) {
    error("write error");
  }

  // So that loop() will skip until it's time to take another
  timeForSample = false;
}

void onAlarm() {
  ++counter;
  if (counter == betweenSamples) {
    timeForSample = true;
    counter = 0;
    digitalWrite(LED_BUILTIN, HIGH);   
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
  Serial.print(counter);
  Serial.print(' ');
}

void loop(void) {
  // Temp sensor uses I2C which doesn't work reliably during an interrupt,
  // so initiate sampling here, immediately after the last interrupt.
  if (timeForSample) {
    takeSample();

    // don't do this forever, the time it takes to print will increase
    // printStoredSamples();
  }  
}
