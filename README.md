# 📡 Fake Weather Broadcaster – ROS 2 Humble

This ROS 2 package publishes synthetic 24-hour weather data using a custom message type.
The publisher runs at **1 Hz**, and each message represents **15 minutes** of virtual time.
A full “virtual day” is **96 samples** (96 × 15 min = 24 hours), after which the cycle repeats.

Includes:

* `weather_interfaces` — custom message definition
* `weather_publisher` — Python publisher node

Works on all **standard ROS 2 Humble installations** (Ubuntu 22.04 recommended).

---

# 🚀 Setup Instructions (Standard ROS 2 Humble)

## 1. Create a workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

## 2. Clone this repository

```bash
git clone <your-repo-url>
```

Your workspace structure should now look like:

```
ros2_ws/
  src/
    FAKE_DATA_BROADCASTER/
      weather_interfaces/
      weather_publisher/
```

## 3. Build the workspace

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

# ▶️ Running the Publisher

### Terminal 1 – start the publisher

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
source install/setup.bash

ros2 run weather_publisher weather_publisher
```

You will see logs like:

```
[t= 2.5 h] T=18.7 °C, H=68.3 %, Wind=1.2 m/s @ 93°
```

---

# 🔍 Viewing Weather Data

### Terminal 2 – echo the topic

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
source install/setup.bash

ros2 topic list
ros2 topic echo /weather
```

Example output:

```text
temperature_c: 21.4
humidity_percent: 47.2
wind_speed_mps: 3.1
wind_direction_deg: 182.0
---
```

---

# 🌤️ How the Fake Weather Works

* Publish rate: **1 message per second (1 Hz)**
* Each message simulates **15 minutes** of real world time
* 96 messages = **24 hours**
* Temperature follows a natural day–night sinusoid
* Humidity inversely follows temperature
* Wind speed peaks in the afternoon
* Wind direction drifts smoothly
* After 24 hours → **loop back to midnight**



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
