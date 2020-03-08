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
vbat_input = AnalogIn(board.VOLTAGE_MONITOR)
dvdr_input = AnalogIn(board.A5) # voltage divider with 2 equal resistors (10kOhm)

# Set up I2C and SPI for peripheral connections
i2c = busio.I2C(board.SCL, board.SDA)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

# Initialize peripherals
sht = adafruit_sht31d.SHT31D(i2c)
rtc = adafruit_pcf8523.PCF8523(i2c)

# SD connection, filesystem, mount point
cs = digitalio.DigitalInOut(board.D10)
sdcard = adafruit_sdcard.SDCard(spi, cs)
vfs = storage.VfsFat(sdcard)
storage.mount(vfs, "/sd")
sd_test_file = "/sd/testfile.txt"

# Overwrite existing file, effectively wiping old data
with open(sd_test_file, "w") as f:
    f.write("Hello world!\r\n")

# Lookup table for names of days (nicer printing).
days = ("Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday")

# Configure averaging for sensor readings
NUM_TEMP_SAMPLES = 5
TIME_BET_SAMPLES = 1

def print_directory(path, tabs=0):
    for file in os.listdir(path):
        stats = os.stat(path + "/" + file)
        filesize = stats[6]
        isdir = stats[0] & 0x4000

        if filesize < 1000:
            sizestr = str(filesize) + " by"
        elif filesize < 1000000:
            sizestr = "%0.1f KB" % (filesize / 1000)
        else:
            sizestr = "%0.1f MB" % (filesize / 1000000)

        prettyprintname = ""
        for _ in range(tabs):
            prettyprintname += "   "
        prettyprintname += file
        if isdir:
            prettyprintname += "/"
        print('{0:<40} Size: {1:>10}'.format(prettyprintname, sizestr))

        # recursively print directory contents
        if isdir:
            print_directory(path + "/" + file, tabs + 1)

def get_voltage(pin):
    return (pin.value * 3.3) / 65536 * 2 #actaul Aref is more like 3.286

if False:   # change to True if you want to set the time!
    #                     year, mon, date, hour, min, sec, wday, yday, isdst
    t = time.struct_time((2020,  02,   25,   16,  12,  30,    2,   -1,    -1))
    # you must set year, mon, date, hour, min, sec and weekday
    # yearday is not supported, isdst can be set but we don't do anything with it at this time
    print("Setting time to:", t)     # uncomment for debugging
    rtc.datetime = t

while True:

#     print("\nFiles on filesystem:")
#     print("====================")
#     print_directory("/sd")

#     print("\nContents of {}:".format(sd_test_file))
#     print("====================")

    # read and print one line at a time, no memory risk
#     with open(sd_test_file, "r") as f:
#         line = f.readline()
#         while line != '':
#             print(line, end='')
#             line = f.readline()

    # read whole file into memory, could overflow
#     with open(sd_test_file, "r") as f:
#         lines = f.readlines()
#         for line in lines:
#             print(line, end='')

    # averaged temps
    sht_cumm_samples = 0
    for i in range(NUM_TEMP_SAMPLES):
        sht_cumm_samples += sht.temperature
        time.sleep(TIME_BET_SAMPLES)

    sht_temperature_c = sht_cumm_samples / NUM_TEMP_SAMPLES
    sht_humidity_pcnt = sht.relative_humidity

    time_date = rtc.datetime

    battery_v = get_voltage(vbat_input)
    divider_v = get_voltage(dvdr_input)

    print("The date is: {}, {}-{:02}-{:02}".format(days[int(time_date.tm_wday)], time_date.tm_year, time_date.tm_mon, time_date.tm_mday))
    print("The time is: {}:{:02}:{:02}".format(time_date.tm_hour, time_date.tm_min, time_date.tm_sec))
    print("Epoch    is: {}".format(time.mktime(time_date)))

    print("The VBat is: {:.2f}".format(battery_v))
    print("The VReg is: {:.2f}".format(divider_v))

    print("SHT31D Temp: {:0.3f} C".format(sht_temperature_c))
    print("SHT31D Relh: {:0.1f} %".format(sht_humidity_pcnt))

    # Write sample string, e.g.: 1583575730,19.489,48.5
    with open(sd_test_file, "a") as f:
        print("Saving to SD\n")
        f.write("{:02}-{:02}_{}:{:02}:{:02},{},{:0.3f},{:0.1f},{:.2f}\r\n".format(
            time_date.tm_mon,
            time_date.tm_mday,
            time_date.tm_hour,
            time_date.tm_min,
            time_date.tm_sec,
            time.mktime(time_date),
            sht_temperature_c,
            sht_humidity_pcnt,
            battery_v))

    # Plotter color order: Green, Blue, Orange, Grey
#     print((sht_temperature_c, sht_humidity_pcnt, battery_v))