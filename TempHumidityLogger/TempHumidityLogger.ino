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

#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

//#define CFG_DEBUG true

// BLE Service
BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery

////////////////////////////////////////////////////////////////////////////////
// Dev selections for switching between board types, etc.
//
#define SENSOR_NAME "Vent1" // used for log filename, five characters or less
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
#define FILE_BASE_NAME SENSOR_NAME "_"

#ifdef ARDUINO_FEATHER_M4 // Has SHT31 backpack
#define BOARDNAME "Adafruit Feather M4 Express"
#define TIMESINCECOMPILE 8 // Lag between compile __TIME__, and setting RTC
#define VREFMULTIPLIER AR_INTERNAL2V4 // 0..2.4v at divider, so 4.8v capable
#define PIN_VBAT A6
#endif

#ifdef ARDUINO_NRF52840_FEATHER_SENSE
#define BOARDNAME "Adafruit Feather nRF52840 Sense"
#define TIMESINCECOMPILE 13 // Lag between compile __TIME__, and setting RTC
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

#ifdef USESERIAL1
delay(500); // FTDI
#else
while (!Serial); // for nrf52840 with native usb
#endif

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
    // while (1); // this will block the ability to set time after power off
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

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behaviour, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(true);

  // Config the peripheral connection with maximum bandwidth
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName(SENSOR_NAME);
  //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Start BLE Battery Service
  blebas.begin();
  blebas.write(100);

  // Set up and start advertising
  startAdv();

  MYSERIAL.println("Please use Adafruit's Bluefruit LE app to connect in UART mode");
  MYSERIAL.println("Once connected, enter character(s) that you wish to send");

  // We're done with setup stuff, now start the interrupt loop.
  enableRTC();
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   *
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  MYSERIAL.print("Connected to ");
  MYSERIAL.println(central_name);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  MYSERIAL.println();
  MYSERIAL.print("Disconnected, reason = 0x"); MYSERIAL.println(reason, HEX);
}

void fowardBleuart() {
  // Forward data from HW Serial to BLEUART
  while (MYSERIAL.available())
  {
    // Delay to wait for enough input, since we have a limited transmission buffer
    delay(2);

    uint8_t buf[64];
    int count = MYSERIAL.readBytes(buf, sizeof(buf));
    bleuart.write( buf, count );
  }

  // Forward from BLEUART to HW Serial
  while ( bleuart.available() )
  {
    uint8_t ch;
    ch = (uint8_t) bleuart.read();
    MYSERIAL.write(ch);
  }
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
  MYSERIAL.print(F("RTC time is: "));
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
  sprintf(csv, "%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", now.unixtime(), voltage,
          relh_sht31, temp_sht31, tmp_bmp280, bar_bmp280, alt_bmp280);

  // Write data to SD file, note string already ends with a new line
  file.println(csv);

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

  // Write RTC and data string to Bleuart
  // char timeStr[16];
  // sprintf(timeStr, "RTC: %02hd:%02hd:%02hd", now.hour(), now.minute(), now.second());
  // bleuart.print(timeStr);
  // bleuart.print(csv);

  // Write data plot to Bleuart
  char plotStr[32];
  // sprintf(plotStr, "%.2f,%.2f,%.2f,%.2f", voltage, relh_sht31, temp_sht31, tmp_bmp280);
  sprintf(plotStr, "%.2f,%.2f,%.2f", voltage, relh_sht31, temp_sht31); // no extra bmp temp
  bleuart.println(plotStr); // needs a new line to plot

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

  fowardBleuart();

}
