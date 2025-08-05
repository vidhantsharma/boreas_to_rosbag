# Boreas Dataset ROS2 Integration

This repository contains tools and configurations for working with the Boreas dataset in a ROS2 environment. It includes scripts for converting Boreas data into ROS2 bag files, visualization configurations, and calibration files.

---

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [How to Run](#how-to-run)
- [File Structure](#file-structure)
- [Scripts](#scripts)
- [Visualization](#visualization)
- [License](#license)

---

## Overview

The Boreas dataset is a multi-sensor dataset designed for autonomous vehicle research. This repository provides tools to convert Boreas data into ROS2-compatible formats for easier integration with ROS2-based systems.

---

## Prerequisites

Before using this repository, ensure you have the following installed:

- Python 3.8+
- ROS2 Humble
- Required Python packages:
  - `numpy`
  - `pandas`
  - `opencv-python`
  - `tqdm`
  - `rclpy`
  - `rosbag2_py`
  - `tf_transformations`
  - `ament_index_python`

Install the Python dependencies using:
```bash
pip install numpy pandas opencv-python tqdm rclpy tf-transformations
```

---

## How to Run

1. **Convert Boreas Dataset to ROS2 Bag File**:
   Use the provided script to convert the Boreas dataset into a ROS2 bag file:
   ```bash
   python scripts/boreas_to_rosbag.py
   ```

2. **Play the ROS2 Bag File**:
   Use the following command to play the generated ROS2 bag file:
   ```bash
   ros2 bag play bags/boreas_ros2_bag_<timestamp>/ --qos-profile-overrides-path tf_static_qos.yaml
   ```

3. **Visualize in RViz**:
   Open the RViz configuration file to visualize the data:
   ```bash
   rviz2 -d rviz/rviz.rviz
   ```

---

## File Structure

```
boreas/
├── boreas-2021-01-26-11-22/       # Dataset folder (example with one of the available sequence)
│   ├── applanix/                  # IMU and GPS data
│   ├── calib/                     # Calibration files
│   ├── lidar/                     # Lidar data
│   ├── radar/                     # Radar data
│   └── camera/                    # Camera data
├── scripts/                       # Conversion scripts
│   └── boreas_to_rosbag.py        # Script to convert Boreas data to ROS2 bag
├── rviz/                          # RViz configuration
│   └── rviz.rviz                  # RViz visualization settings
├── tf_static_qos.yaml             # QoS settings for tf_static
├── .gitignore                     # Git ignore file
└── README.md                      # Project documentation
```

---

## Scripts

### `boreas_to_rosbag.py`

This script converts the Boreas dataset into a ROS2 bag file. It processes data from lidar, radar, camera, and GPS/IMU sensors and writes them into a ROS2-compatible format.

#### Usage:
```bash
python scripts/boreas_to_rosbag.py
```

#### Parameters:
- `base_path`: Path to the Boreas dataset folder. (eg ->  base_path = r"boreas-2021-01-26-11-22")
- `bag_length`: Length of the ROS2 bag file in seconds (default: 60 seconds). Set it to '-1' if whole of the sequence is to be converted.

---

## Visualization

The RViz configuration file (`rviz/rviz.rviz`) is pre-configured to visualize the following topics:
- `/lidar`: Lidar point cloud data
- `/camera/image_raw`: Camera images
- `/radar/image_raw`: Radar images
- `/tf`: Transform frames
- `/odometry`: Odometry data

To launch RViz with this configuration:
```bash
rviz2 -d rviz/rviz.rviz
```

---