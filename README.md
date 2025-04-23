# ROS2-Pallet-detection-and-ground-segmentation

A ROS2 package for real-time pallet detection and ground segmentation using YOLOv11 models.

## Overview
This package provides two ROS2 nodes:

- Pallet Detection: Identifies and draws bounding boxes around pallets in camera feed using YOLOv11m
- Ground Segmentation: Segments and highlights the ground surface in camera feed using YOLOv11 segment

Both nodes use custom trained YOLOv11 models to perform computer vision tasks on ZED2i camera images, making them suitable for mobile robot navigation and manipulation applications.

## Features

- Real-time pallet detection using a custom YOLOv11m model
- Ground surface segmentation with color overlay using YOLOv11 segment
- Integration with ZED2i stereoscopic camera
- Optimized QoS profiles for reliable performance
- Visualization outputs for debugging and monitoring

## Prerequisites

- ROS2 (tested on Humble)
- Python 3.8+
- CUDA-capable GPU (recommended)
- ZED2i camera

## Dependencies
ROS2 packages
```
ros-humble-cv-bridge
ros-humble-sensor-msgs
ros-humble-vision-msgs
```
Python packages
```
pip install ultralytics
pip install opencv-python
pip install numpy
```

## Installation
Clone this repository:
```
git clone https://github.com/ChandhanSaai
```
Open the node file to modify:
```
# For pallet detection
/home/chandhan/pallet_ws/src/pallet_segmentation/pallet_segmentation/inference.py

# For ground segmentation
/pallet_ws/src/pallet_segmentation/pallet_segmentation/inference_segment.py
```

The trained YOLOv11 models are included with the package. You'll just need to update the model paths in the code to match your system



