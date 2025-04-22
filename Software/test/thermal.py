import time
import busio
import board
import adafruit_amg88xx

# Initialize I2C connection and AMG8833 sensor
try:
    i2c = busio.I2C(board.SCL, board.SDA)
    sensor = adafruit_amg88xx.AMG88XX(i2c)
except Exception as e:
    print(f"Error initializing AMG8833: {e}")
    print("Ensure Adafruit Blinka is installed and I2C is enabled.")
    exit()

print("AMG8833 Thermal Sensor Initialized.")
print("Reading temperatures...")

try:
    while True:
        # Read and print the 8x8 temperature grid
        for row in sensor.pixels:
            print(["{:.1f}".format(temp) for temp in row])
        print("\n")  # Add a blank line between readings
        time.sleep(1)  # Delay for 1 second between readings

except KeyboardInterrupt:
    print("Exiting...")

