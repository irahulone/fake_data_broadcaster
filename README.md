# Fake Weather Broadcaster – ROS 2 Humble

This ROS 2 package publishes synthetic 24-hour weather data using a custom message type.
The publisher operates at **1 Hz**, and each message represents **15 minutes** of simulated time.
A complete cycle consists of **96 samples** (96 × 15 minutes = 24 hours), after which the pattern repeats.

The package includes:

* `weather_interfaces` — custom `.msg` definition
* `weather_publisher` — publisher that outputs all weather fields on a single topic (`/weather`)
* `weather_split_publisher` — publisher that outputs the four weather variables on separate topics

This package is compatible with standard ROS 2 Humble installations (Ubuntu 22.04 recommended).

---

## Setup Instructions (ROS 2 Humble, Standard Installation)

### 1. Create a workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clone this repository

```bash
git clone <your-repo-url>
```

Your workspace structure should resemble:

```
ros2_ws/
  src/
    FAKE_DATA_BROADCASTER/
      weather_interfaces/
      weather_publisher/
      weather_split_publisher/
```

### 3. Build the workspace

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## Running the Combined Weather Publisher (`/weather`)

### Terminal 1

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
source install/setup.bash

ros2 run weather_publisher weather_publisher
```

Example log output:

```
[t= 2.5 h] T=18.7 °C, H=68.3 %, Wind=1.2 m/s @ 93°
```

---

## Viewing Combined Weather Data

### Terminal 2

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
source install/setup.bash

ros2 topic list
ros2 topic echo /weather
```

Example message:

```text
temperature_c: 21.4
humidity_percent: 47.2
wind_speed_mps: 3.1
wind_direction_deg: 182.0
---
```

---

## Separate Topic Publisher (`weather_split_publisher`)

The `weather_split_publisher` node publishes the four weather variables as individual topics using `std_msgs/Float32`.

| Topic Name        | Message Type       | Description               |
| ----------------- | ------------------ | ------------------------- |
| `/temperature`    | `std_msgs/Float32` | Temperature in °C         |
| `/humidity`       | `std_msgs/Float32` | Relative humidity (%)     |
| `/wind_speed`     | `std_msgs/Float32` | Wind speed in m/s         |
| `/wind_direction` | `std_msgs/Float32` | Wind direction in degrees |

### Running the Split Publisher

### Terminal 1

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
source install/setup.bash

ros2 run weather_split_publisher weather_split_publisher
```

### Terminal 2 (view individual topics)

```bash
ros2 topic echo /temperature
ros2 topic echo /humidity
ros2 topic echo /wind_speed
ros2 topic echo /wind_direction
```

---

## Synthetic Weather Model

* Publish rate: 1 Hz
* Each published message corresponds to 15 minutes of simulated time
* A full cycle consists of 96 messages (24 hours)
* Temperature follows a sinusoidal day–night cycle
* Humidity varies inversely with temperature
* Wind speed tends to increase during the afternoon
* Wind direction drifts gradually with added noise
* After 24 hours, the data repeats from the beginning


<!-- ---

# 🍎 2. macOS + micromamba + ROS 2 Humble

> **Note:** Only needed for *macOS (M1/M2/M3)* using **RoboStack Humble** + **Python 3.9**.
> Regular ROS 2 users should ignore this section.

## Activate environment

```bash
micromamba activate ros_humble_py39
cd ~/ros2_ws
```

---

## Build (required when anything changes under `src/`)

macOS + micromamba requires explicitly forcing CMake to use Python 3.9:

```bash
colcon build --cmake-args \
  "-DPython3_ROOT_DIR=$CONDA_PREFIX" \
  "-DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python" \
  "-DPython3_INCLUDE_DIR=$CONDA_PREFIX/include/python3.9" \
  "-DPython3_NumPy_INCLUDE_DIRS=$CONDA_PREFIX/lib/python3.9/site-packages/numpy/core/include"
```

Source the workspace:

```bash
source install/setup.zsh
```

---

## Run the publisher (Terminal 1)

```bash
micromamba activate ros_humble_py39
cd ~/ros2_ws
source install/setup.zsh

ros2 run weather_publisher weather_publisher
```

---

## Inspect the data (Terminal 2)

```bash
micromamba activate ros_humble_py39
cd ~/ros2_ws
source install/setup.zsh

ros2 topic list
ros2 topic echo /weather
```

--- -->

<!-- # 🧪 How the Fake Weather Works

* Publish rate: **1 Hz**
* Each publish = **15 min virtual time**
* 96 samples = **24 hours**
* Temperature varies with a daily sinusoid
* Humidity inversely mirrors temperature
* Wind speed peaks in afternoon
* Wind direction drifts gently
* After 96 samples → loops back to midnight
 -->
