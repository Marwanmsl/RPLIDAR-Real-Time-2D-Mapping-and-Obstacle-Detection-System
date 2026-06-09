# RPLIDAR Real-Time 2D Mapping and Obstacle Detection System

## Overview

https://cloud.githubusercontent.com/assets/61601234//2bee1e88-8bc9-4a12-81b3-3e41fa98d30e.mp4

This project implements a real-time 2D environment mapping and obstacle detection system using an RPLIDAR sensor and Python. The system continuously collects LiDAR scan data, converts polar measurements into Cartesian coordinates, builds an occupancy map, identifies obstacles through clustering techniques, and displays the results in a full-screen graphical interface.

The application is designed for robotics, autonomous navigation, indoor mapping, collision avoidance, and research applications.

## Features

### Real-Time 2D Mapping

* Generates a live occupancy map from RPLIDAR scan data.
* Converts polar coordinates into Cartesian map coordinates.
* Supports persistent mapping with optional trail visualization.

### Obstacle Detection

* Detects objects using distance-based point clustering.
* Calculates:

  * Object center position
  * Width and height
  * Distance from LiDAR
  * Number of detected points

### Collision Warning System

* Continuously monitors nearest detected obstacle.
* Displays visual warning when an obstacle enters a predefined safety zone.

### Interactive Visualization

* Full-screen OpenCV display.
* Real-time map updates.
* Grid overlay for spatial reference.
* LiDAR position visualization.
* Obstacle highlighting and distance labeling.

### Map Export

* Automatically saves the generated occupancy map as an image when the program exits.

## Hardware Requirements

* RPLIDAR A1 / A2 / A3
* USB connection to PC
* Windows/Linux computer
* Python 3.8+

## Software Requirements

### Python Libraries

```bash
pip install rplidar-roboticia
pip install numpy
pip install opencv-python
```

Additional standard libraries:

```python
math
tkinter
```

## System Architecture

RPLIDAR
↓
Scan Acquisition
↓
Polar Coordinate Processing
↓
Occupancy Map Generation
↓
Point Clustering
↓
Obstacle Detection
↓
Collision Warning
↓
Real-Time Visualization

## How It Works

### 1. LiDAR Scan Acquisition

The system continuously receives scan measurements from the RPLIDAR.

Each measurement contains:

* Quality
* Angle (degrees)
* Distance (mm)

Example:

```python
(quality, angle, distance)
```

### 2. Coordinate Conversion

Polar coordinates are converted into Cartesian coordinates.

```python
x = distance * cos(angle)
y = distance * sin(angle)
```

These coordinates are transformed into map pixels for visualization.

### 3. Occupancy Mapping

Detected points are projected onto a 2D occupancy grid.

Each occupied location is marked on the map:

```python
occupancy_map[py, px] = 255
```

### 4. Obstacle Clustering

Neighboring points are grouped into clusters using Euclidean distance.

If the distance between points is below a threshold:

```python
CLUSTER_DISTANCE = 200 mm
```

they are considered part of the same obstacle.

### 5. Obstacle Analysis

For every cluster, the system calculates:

* Center position
* Width
* Height
* Distance from sensor
* Point count

This information helps estimate object size and location.

### 6. Safety Monitoring

The nearest obstacle is continuously tracked.

When an object enters:

```python
WARNING_DISTANCE = 1000 mm
```

the system displays:

```
WARNING: OBSTACLE CLOSE!
```

## Configuration Parameters

| Parameter          | Description                          |
| ------------------ | ------------------------------------ |
| MAP_SIZE           | Occupancy map size                   |
| MAP_RESOLUTION     | mm per pixel                         |
| MIN_DISTANCE       | Minimum valid LiDAR distance         |
| MAX_DISTANCE       | Maximum valid LiDAR distance         |
| CLUSTER_DISTANCE   | Distance for obstacle clustering     |
| MIN_CLUSTER_POINTS | Minimum points required for obstacle |
| WARNING_DISTANCE   | Collision warning threshold          |

---

## Controls

| Key | Function            |
| --- | ------------------- |
| ESC | Exit application    |
| C   | Clear occupancy map |

---

## Output

### Live Display

* Occupancy map
* Grid system
* LiDAR location
* Obstacle boundaries
* Distance labels
* Warning messages

### Saved File

```text
rplidar_obstacle_map.png
```

---

## Applications

### Robotics

* Mobile robot navigation
* Autonomous vehicles
* Warehouse robots

### Research

* SLAM development
* Obstacle avoidance testing
* LiDAR data analysis

### Industrial

* Safety monitoring
* Autonomous inspection systems
* Smart factory navigation

## Future Improvements

* ROS2 integration
* SLAM implementation
* Path planning algorithms
* Object classification using AI
* Dynamic obstacle tracking
* Multi-sensor fusion (LiDAR + Camera)
* 3D mapping support
* Autonomous navigation system

## Author

Mohammed Marwan

AI Engineer | Robotics Engineer | Computer Vision Developer

Specializing in:

* Artificial Intelligence
* Autonomous Robotics
* LiDAR Mapping
* Computer Vision
* UAV Systems
* Embedded AI
