# ROS2-Pallet-detection-and-ground-segmentation

A ROS2 package for real-time pallet detection and ground segmentation using YOLOv11 models.

## Overview
This package provides two ROS2 nodes:

- Pallet Detection: Identifies and draws bounding boxes around pallets in camera feed using YOLOv11m
- Ground Segmentation: Segments and highlights the ground surface in camera feed using YOLOv11 segment

Both nodes use custom trained YOLOv11 models to perform computer vision tasks on ZED2i camera images, making them suitable for mobile robot navigation and manipulation applications.
