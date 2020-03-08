import time
import board
import busio
import digitalio
from analogio import AnalogIn
import os
import adafruit_sdcard
import storage

import adafruit_pcf8523
import adafruit_sht31d

# Get battery and system voltage
vbat_voltage = AnalogIn(board.VOLTAGE_MONITOR)
dvdr_voltage = AnalogIn(board.A5) # voltage divider with 2 equal resistors (220 Ohm)

# Create library object using our Bus I2C port
i2c_bus = busio.I2C(board.SCL, board.SDA)

# Initialize peripherals
sht = adafruit_sht31d.SHT31D(i2c_bus)

# Configure averaging for sensor readings
NUM_TEMP_SAMPLES = 5
TIME_BET_SAMPLES = 1


def get_voltage(pin):
    return (pin.value * 3.3) / 65536 * 2 #actaul Aref is more like 3.286

while True:

    # averaged temps
    sht_cumm_samples = 0
    for i in range(NUM_TEMP_SAMPLES):
        sht_cumm_samples += sht.temperature
        time.sleep(TIME_BET_SAMPLES)

    sht_temperature_c = sht_cumm_samples / NUM_TEMP_SAMPLES
    sht_humidity_pcnt = sht.relative_humidity

    print("\nThe VReg is: {:.2f}".format(get_voltage(dvdr_voltage)))
    print("The VBat is: {:.2f}".format(get_voltage(vbat_voltage)))

    print("\nSHT31D Temperature: {:0.3f} C".format(sht_temperature_c))
    print("SHT31D Humidity: {:0.1f} %".format(sht_humidity_pcnt))
    print("\n")

    # Plotter color order: Green, Blue, Orange, Grey
    print((sht_temperature_c, sht_humidity_pcnt))