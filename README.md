# **Fake Weather Broadcaster – ROS 2 Humble**

This ROS 2 package publishes synthetic, time-varying weather data on **four separate ROS topics**.
The publisher runs at **1 Hz**, and each message represents **15 minutes** of simulated time.
A full cycle consists of **96 samples** (96 × 15 min = 24 hours) before repeating.

This repository contains a single ROS 2 Python package:

* **`weather_data_publisher`** – publishes temperature, humidity, wind speed, and wind direction as **individual topics**.

This package works on standard ROS 2 Humble installations (Ubuntu 22.04 recommended).
(Mac users can also run it with a compatible ROS 2 environment.)

---

# **Setup Instructions**

## 1. Create a workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

## 2. Clone this repository

```bash
git clone <your-repo-url>
```

Your workspace should look like:

```
ros2_ws/
  src/
    FAKE_DATA_BROADCASTER/
      weather_data_publisher/
      package.xml
      setup.py
      setup.cfg
      resource/
```

## 3. Build the workspace

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

# **Running the Weather Data Publisher**

The `weather_data_publisher` node publishes the following topics:

| Topic             | Type               | Description              |
| ----------------- | ------------------ | ------------------------ |
| `/temperature`    | `std_msgs/Float32` | Temperature (°C)         |
| `/humidity`       | `std_msgs/Float32` | Relative humidity (%)    |
| `/wind_speed`     | `std_msgs/Float32` | Wind speed (m/s)         |
| `/wind_direction` | `std_msgs/Float32` | Wind direction (degrees) |

### Terminal 1 — Run the node

Your executable name is:

```
weather_data_publisher_node
```

Run it with:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run weather_data_publisher weather_data_publisher_node
```

---

# **Viewing Weather Data**

### Terminal 2 — Echo individual topics

```bash
source ~/ros2_ws/install/setup.bash

ros2 topic echo /temperature
ros2 topic echo /humidity
ros2 topic echo /wind_speed
ros2 topic echo /wind_direction
```

---

# **Synthetic Weather Model**

The data generator simulates a smooth 24-hour weather cycle:

* Publish rate: **1 Hz**
* Each sample = **15 minutes** simulated time
* **96 samples = 24-hour cycle**
* Temperature follows a sinusoidal day/night pattern
* Humidity varies inversely with temperature
* Wind speed tends to peak midday
* Wind direction drifts slowly with noise
* After *one full cycle*, the pattern repeats


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
