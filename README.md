# NXP Line Following Robot

An autonomous 2line-following robot built on the Teensy microcontroller with advanced vision-based steering using Pixy2 camera. Designed for precision line tracking with intersection detection and obstacle avoidance.

## Project Overview

This project implements a sophisticated autonomous robot controller that:
- Follows lines detected by Pixy2 camera vision system
- Intelligently handles intersections and corners
- Detects and responds to obstacles using ultrasonic distance sensor
- Recognizes finish line patterns and stops safely
- Applies advanced steering algorithms for smooth, stable tracking

## Hardware Components

### Microcontroller
- **Teensy** (primary controller)

### Sensors
- **Pixy2 Camera** - Line detection and vector tracking
- **HC-SR04 Ultrasonic Sensor** - Distance measurement for obstacle detection
  - TRIG: Pin 14
  - ECHO: Pin 15

### Actuators
- **Servo Motor** - Steering control (Pin 23)
- **DC Motors (x2)** - Left and right propulsion
  - Left Motor: PWM (Pin 8), Direction (Pins 6-7)
  - Right Motor: PWM (Pin 18), Direction (Pins 20-19)

### Additional
- **LED Indicator** - Pin 22
- **Logic Level Converters** - For Pixy2 communication

## Pin Configuration

```
PWM1 = 8      (Left motor speed)
IN1 = 6       (Left motor direction)
IN2 = 7       (Left motor direction)
PWM2 = 18     (Right motor speed)
IN3 = 20      (Right motor direction)
IN4 = 19      (Right motor direction)

LED = 22
ECHO = 15     (Ultrasonic echo)
TRIG = 14     (Ultrasonic trigger)
SERVO_PIN = 23 (Steering servo)
```

## Key Features

### 1. Advanced Line Following
- Processes up to 20 line vectors from Pixy2 camera
- Weighted steering algorithm that considers vector angles and lengths
- Minimum vector length filtering to ignore noise
- Soft steering limits to prevent oscillation

### 2. Intersection Detection
- Identifies corner types: top-left, top-right, bottom-left, bottom-right
- Analyzes angle between consecutive vectors (target ~90°)
- Applies directional corrections based on intersection position
- More aggressive steering during turns for accuracy

### 3. Steering Control
- **Center Angle:** 135° (neutral position)
- **Steering Range:** 90° (MAX_LEFT) to 180° (MAX_RIGHT)
- **Deadband:** ±15° around forward angle to filter horizontal noise
- **Soft Limiting:** Reduces extreme steering angles to prevent oscillation
- **Center Bias:** Gently pulls steering toward center for stability
- **Delay System:** Smooths steering inputs over 2 cycles

### 4. Obstacle Detection & Avoidance
- Continuously monitors distance via ultrasonic sensor
- Filtered readings (average of 3 samples) for stability
- **Stop Distance:** 15 cm - triggers reverse and stop
- Timeout handling for cases where no object is detected

### 5. Finish Line Recognition
- Detects horizontal line patterns (angle ≤ 15° from horizontal)
- Requires 2+ horizontal vectors within 5 pixels of each other
- Activation delay (10 seconds) to avoid false positives at start
- Triggers acceleration boost and obstacle check

## Build & Upload

This project uses **PlatformIO** for build and upload management.

### Prerequisites
- VSCode with PlatformIO extension
- Arduino IDE libraries (Pixy2, Servo)

### Building
```bash
pio run
```

### Uploading to Device
```bash
pio run --target upload
```

### Monitoring Serial Output
```bash
pio device monitor
```

## Algorithm Flow

1. **Initialization**
   - Configure PWM frequencies (20 kHz)
   - Initialize pins and servo to center
   - Initialize Pixy2 in line-tracking mode
   - Record start time for finish line delay

2. **Main Loop (runNXP)**
   - Read Pixy2 vectors
   - Filter vectors by length, angle, and position
   - Identify horizontal vectors for finish line detection
   - Sort vectors by length (longest first)
   - Compute steering angle based on vector count:
     - **0 vectors:** Maintain last angle
     - **1 vector:** Standard line following
     - **2+ vectors:** Check for intersection; if ~90° angle detected, apply intersection steering
   - Apply steering corrections (center bias, soft limits)
   - Implement steering delay system for smooth transitions
   - Check finish line and obstacle avoidance conditions

3. **Distance Monitoring**
   - Triggered after finish line detection
   - Backs up and stops if obstacle within STOP_DISTANCE

## Constants & Tuning Parameters

```cpp
MAX_VECTORS = 20          // Max vectors to process per frame
MIN_VECTOR_LENGTH = 10    // Minimum pixels for valid vector

CENTRE_ANGLE = 135        // Neutral steering position
MAX_RIGHT = 180           // Maximum right turn
MAX_LEFT = 90             // Maximum left turn

STOP_DISTANCE = 15        // cm - obstacle stop threshold

PIXY_FORWARD_ANGLE = 270  // Expected angle for straight tracking
PIXY_STEER_DEADBAND = 15  // Angle tolerance for horizontal detection
STEERING_DELAY_CYCLES = 2 // Smoothing delay for steering changes
STEERING_QUEUE_THRESHOLD = 2.0   // Threshold to queue steering change
STEERING_CENTER_BIAS = 5.0       // Pull toward center amount
STEERING_SOFT_LIMIT_START = 11   // When to start softening
STEERING_SOFT_LIMIT_FACTOR = 0.5 // Softening strength
CROSS_CORRECTION_ANGLE = 15      // Intersection correction
```

## Performance Notes

- **Steering Response:** Balanced between responsiveness and stability
- **Motor Speed:** Default 150 units for forward motion, 130 for finish line
- **Reverse Speed:** 100 units for obstacle avoidance
- **Frame Rate:** Dependent on Pixy2 refresh rate (typical ~50 FPS)

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Robot doesn't follow line | Check Pixy2 is in "line" mode; verify camera focus; check lighting |
| Erratic steering | Increase STEERING_DELAY_CYCLES; adjust STEERING_CENTER_BIAS |
| Misses turns | Lower angle threshold in intersection detection; increase CROSS_CORRECTION_ANGLE |
| Oscillates | Enable soft limiting; increase STEERING_SOFT_LIMIT_FACTOR |
| Overshoots finish line | Reduce finish line acceleration value; increase STOP_DISTANCE |

## Project Structure

```
NXP/
├── src/
│   ├── main.cpp          # Main robot controller
│   └── README.md         # This file
├── include/
├── lib/
├── platformio.ini        # Build configuration
└── test/
```

## Future Enhancements

- PID control for smoother steering response
- Dynamic speed adjustment based on turn sharpness
- Gyro integration for improved angle estimation
- Adaptive threshold tuning based on lighting conditions
- Multi-point finish line detection for reliability

---

**Last Updated:** June 2026  
**Board:** Teensy  
**Camera:** Pixy2  
**Competition:** NXP
