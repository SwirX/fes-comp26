# Fes-Comp2026 Robot Controller

A modular, high-performance robotics architecture utilizing an ESP32 for direct motor manipulation via continuous rotation servos, an ESP32-CAM for low-latency visual data streams (MJPEG), and a Python controller for Computer Vision analysis using YOLOv8/v11 and PyZbar.

## Components
- `esp32_motors/`: Handles UDP control datagrams and converts them into servo motions. Implements failsafe mechanisms.
- `esp32_cam/`: Serves video streams and acknowledges UDP heartbeats.
- `laptop_client/`: Central control hub for video feed manipulation, YOLO sign detection, and keyboard input processing to direct the motors.
- `yolo_training/`: Tools for model fine-tuning specifically on Moroccan road signs (stop, wrong-way, speed limits, etc.).
