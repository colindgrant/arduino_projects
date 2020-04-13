/* Simple sketch to try out the 'Comidox' super small break out board,
 *  which appears to be a knockoff of the Adafruit ADS1015 breakout.
 *  
 * Works on Mega.
 */

#include <Adafruit_ADS1015.h>   // Adafruit's ADC breakout as voltage monitor
#include <Wire.h>
 
// ADC used as voltage monitor, I2C address 0x48 HEX, 72 DEC
Adafruit_ADS1015 ads;

void iic_discover() {
  byte error, address;
  int nDevices;

  Serial.println("\nI2C Scanner");
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
 
  delay(5000);           // wait 5 seconds for next scan
}

void setup() {
  Wire.begin();
 
  Serial.begin(115200);
  while (!Serial);             // Leonardo: wait for serial monitor

  Serial.println("Starting ADS1015");
  ads.begin();

//  delay (1000);
  
}

void loop() {
//  iic_discover();

  // A0 = index 1
  // A1 = index 2
  // A2 = index 3
  // A3 = index 0

  uint8_t numMeters = 4; // 4 ADC channels
  uint8_t index; // common loop iterator
  float voltage;
  byte remapInputs[] = {3, 0, 1, 2};

  
  for (index = 0; index < numMeters; index++) {
    delay (10); // required to get actual readings
    voltage = (ads.readADC_SingleEnded(index) * .003); // .003mV per division
    Serial.print("Voltage ");
    Serial.print(remapInputs[index]);
    Serial.print(": ");
    Serial.println(voltage);
  }
  Serial.println();

  delay (500);

}
