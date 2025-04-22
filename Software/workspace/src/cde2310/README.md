# ThermalPublisher Node

This is a ROS 2 Python node that reads data from the AMG8833 8x8 thermal sensor and publishes directional information based on detected heat patterns.

## Overview

The node captures thermal data and analyzes it to determine the direction of the heat source relative to the sensor. It then publishes a corresponding string message to the `thermal_data` topic.

## Published Topic

- **/thermal_data** (`std_msgs/String`): Contains direction info:
  - `N` – No significant heat source detected
  - `S` – Most pixels are above the threshold (saturation)
  - `L` – Heat is primarily on the left side
  - `R` – Heat is primarily on the right side
  - `F{n}` – Heat is centered; `{n}` indicates number of hot pixels in the center

## How It Works

1. The node initializes the AMG8833 sensor using I2C.
2. It captures thermal data every second.
3. On the **first frame**, the ambient temperature is calculated and saved.
4. In subsequent frames, a dynamic threshold (`ambient + 2.5°C`) is used to identify "hot" pixels.
5. Based on the distribution of hot pixels:
   - If fewer than 5% are hot → `N`
   - If more than 50% are hot → `S`
   - Otherwise, direction (`L`, `R`, `F`) is determined by location of hot spots.

## Dependencies

- ROS 2 (Foxy/Humble or later)
- `adafruit-circuitpython-amg88xx`
- `rclpy`
- `board`
- `busio`
- `std_msgs`

Install the sensor library using pip:
```
pip install adafruit-circuitpython-amg88xx
```

## Running the Node

Make sure your I2C sensor is connected and enabled, then run:
```
thermal
```

> [!CAUTION]
> This assumes you have added an alias by following the instructions [here](../../README.md).

> [!NOTE]
> Make sure I2C is enabled on your board (e.g., Raspberry Pi).
> The first frame is used to calibrate the ambient temperature.
> If the node starts too close to a heat source, it may skew initial ambient detection.

## Custom parameter file for Nav2 Stack

The custom parameter file for Nav2 Stack is in the config folder.
The file has custom parameters for robot radius, adjusted costmaps settings and other parameters for smoother mission.
