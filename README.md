# Prosthetic Hand Gesture Recognition System

Real-time hand gesture detection system using MediaPipe and OpenCV to control a robotic prosthetic hand via Arduino servo motors.

## Features

- Real-time Hand Detection (MediaPipe, 99% accuracy)
- Finger Angle Calculation (3D geometry based)
- Arduino Integration (5 servo motors)
- Live Visualization with Real-time Angles
- Interactive Calibration Tool
- Comprehensive Documentation

## Quick Start

1. Install dependencies: `pip install -r python/requirements.txt`
2. Upload Arduino code: `arduino/prosthetic_hand_servo_control.ino`
3. Configure port in `hand_gesture_control.py`
4. Run calibration: `python python/calibration.py`
5. Run main program: `python python/hand_gesture_control.py`

## System Architecture

```
Webcam, MediaPipe, Python Calculation, Serial, Arduino, Servos, Hand
```

## Requirements

- Arduino Uno/Mega/Nano
- 5 Servo Motors (SG90 recommended)
- USB Webcam
- 5V Power Supply (external for servos)
- Python 3.8+

## Configuration

Edit `hand_gesture_control.py`:
- `ARDUINO_PORT`: Your COM port (COM3, /dev/ttyUSB0, etc.)
- `SMOOTHING_FACTOR`: 0.6 (adjust for jitter reduction)
- `MIN_DETECTION_CONFIDENCE`: 0.7 (adjust for sensitivity)

Edit `prosthetic_hand_servo_control.ino`:
- Servo pins (3, 5, 6, 9, 10)
- `DEBUG_MODE`: Set to 1 for debug output

## License

MIT License - Open Source