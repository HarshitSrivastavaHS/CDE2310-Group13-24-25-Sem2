import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import RectBivariateSpline
import busio
import board
import adafruit_amg88xx

# Initialize I2C connection and AMG8833 sensor
try:
    i2c = busio.I2C(board.SCL, board.SDA)
    sensor = adafruit_amg88xx.AMG88XX(i2c)
except AttributeError:
    print("Error: Ensure Adafruit Blinka is installed and configured correctly.")
    print("Run: sudo pip3 install --upgrade --force-reinstall adafruit-blinka Adafruit-PlatformDetect")
    exit()

# Constants for visualization
MINTEMP = 20  # Minimum temperature for color mapping (blue)
MAXTEMP = 40  # Maximum temperature for color mapping (red)
COLORDEPTH = 1024  # Number of color values in the heatmap

# Interpolation settings to enhance resolution
GRID_SIZE = 8  # AMG8833 outputs an 8x8 grid
INTERPOLATED_SIZE = 32  # Size of interpolated grid

# Generate color map for visualization
colors = plt.cm.jet(np.linspace(0, 1, COLORDEPTH))
color_map = plt.cm.ScalarMappable(cmap=plt.cm.jet)
color_map.set_clim(MINTEMP, MAXTEMP)

# Setup figure for real-time plotting
plt.ion()
fig, ax = plt.subplots()
heatmap = ax.imshow(np.zeros((INTERPOLATED_SIZE, INTERPOLATED_SIZE)), cmap='jet', interpolation='bilinear', vmin=MINTEMP, vmax=MAXTEMP)
cbar = fig.colorbar(heatmap)
cbar.set_label('Temperature (°C)')

def interpolate_data(data):
    """Interpolate the 8x8 grid to a higher resolution using RectBivariateSpline."""
    x = np.arange(GRID_SIZE)
    y = np.arange(GRID_SIZE)
    spline = RectBivariateSpline(x, y, data)

    interp_x = np.linspace(0, GRID_SIZE - 1, INTERPOLATED_SIZE)
    interp_y = np.linspace(0, GRID_SIZE - 1, INTERPOLATED_SIZE)

    return spline(interp_x, interp_y)

try:
    while True:
        # Read pixel temperatures from the sensor (8x8 grid)
        pixels = sensor.pixels

        # Interpolate data to enhance resolution
        interpolated_pixels = interpolate_data(np.array(pixels))

        # Update heatmap plot
        heatmap.set_data(interpolated_pixels)
        heatmap.set_clim(vmin=MINTEMP, vmax=MAXTEMP)
        fig.canvas.draw()
        fig.canvas.flush_events()

        # Print raw data to console (optional)
        print("Raw Data:")
        for row in pixels:
            print(["{:.1f}".format(temp) for temp in row])
        
        time.sleep(0.5)  # Delay for real-time updates

except KeyboardInterrupt:
    print("Exiting...")

