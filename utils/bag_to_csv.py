#!/usr/bin/env python3

import os
import csv
import yaml

import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

# ---------- CONFIG ----------
BAG_DIR = "/Users/kangwang/ros2_ws/wind_speed_bag"
OUT_CSV = "/Users/kangwang/ros2_ws/src/fake_data_broadcaster/wind_speed.csv"
TARGET_TOPIC = "/wind_speed"   # change if your topic name is different
# -----------------------------

# 1) Read metadata.yaml to get storage_id
metadata_path = os.path.join(BAG_DIR, "metadata.yaml")
if not os.path.exists(metadata_path):
    raise FileNotFoundError(f"metadata.yaml not found at {metadata_path}")

with open(metadata_path, "r") as f:
    meta = yaml.safe_load(f)

info = meta["rosbag2_bagfile_information"]
storage_id = info["storage_identifier"]

print(f"[INFO] Bag dir:    {BAG_DIR}")
print(f"[INFO] Storage ID: {storage_id}")

# 2) Open the bag
reader = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(uri=BAG_DIR, storage_id=storage_id)
converter_options = rosbag2_py.ConverterOptions("", "")
reader.open(storage_options, converter_options)

topics_and_types = reader.get_all_topics_and_types()
print("[INFO] Topics in bag:")
for t in topics_and_types:
    print(f"  - {t.name} ({t.type})")

# Find the type of the target topic
topic_type = None
for t in topics_and_types:
    if t.name == TARGET_TOPIC:
        topic_type = t.type
        break

if topic_type is None:
    raise RuntimeError(f"Topic {TARGET_TOPIC} not found in bag.")

print(f"[INFO] Exporting topic {TARGET_TOPIC} of type {topic_type}")

# 3) Prepare message type
msg_type = get_message(topic_type)

rows = []

while reader.has_next():
    topic, data, t = reader.read_next()
    if topic != TARGET_TOPIC:
        continue

    # Deserialize binary CDR → ROS 2 message object
    msg = deserialize_message(data, msg_type)

    # For std_msgs/msg/Float64 or similar, the value is in msg.data
    if not hasattr(msg, "data"):
        raise RuntimeError(f"Message type {topic_type} has no 'data' field; "
                           f"adjust the script for its fields.")

    value = msg.data
    time_sec = t / 1e9

    rows.append((time_sec, value))

print(f"[INFO] Read {len(rows)} messages from {TARGET_TOPIC}")

if not rows:
    print("[WARN] No messages found; CSV will not be created.")
    exit(0)

# 4) Write CSV
os.makedirs(os.path.dirname(OUT_CSV), exist_ok=True)

with open(OUT_CSV, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["time_sec", "data"])
    writer.writerows(rows)

print(f"[INFO] Wrote CSV to {OUT_CSV}")
