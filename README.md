# Sign Language Recognition System

A compact Arduino-based system for recognizing sign language gestures using flex sensors and K-Nearest Neighbors (KNN) algorithm.

## Features

- Recognizes 6 sign language gestures (A, B, D, Y, W, L)
- Training mode to create custom gesture profiles
- Recognition mode with confidence level display
- Persistent storage of training data in EEPROM
- User-friendly interface with 16x2 LCD display

## Hardware Requirements

- Arduino Uno R3
- 5 flex sensors (one for each finger)
- 16x2 LCD display with I2C adapter
- 2 push buttons
- Breadboard and connecting wires

## Connections

| Component | Arduino Pin |
|-----------|-------------|
| Thumb Flex Sensor | A0 |
| Index Finger Flex Sensor | A1 |
| Middle Finger Flex Sensor | A2 |
| Ring Finger Flex Sensor | A3 |
| Pinky Flex Sensor | A4 |
| LCD SDA | 8 (custom) |
| LCD SCL | 9 (custom) |
| Mode Button | 11 |
| Select Button | 12 |

## Custom I2C Implementation

This project uses a custom SoftI2C library implementation to allow using non-standard pins (8 & 9) for I2C communication with the LCD display. This approach is useful when:

- The default I2C pins are needed for other purposes
- Multiple I2C devices with address conflicts need to be connected
- Special timing requirements must be met

## How It Works

1. **Menu Mode**: Choose between Training and Recognition modes
2. **Training**: Collect 10 samples for each gesture
3. **Recognition**: System matches current hand position against training data
4. **Controls**:
   - Mode button: Navigate through options
   - Select button: Confirm selection
   - Long-press Select: Return to previous menu

## KNN Algorithm

The system implements K-Nearest Neighbors (K=3) algorithm:
- Calculates distance between current readings and all training samples
- Finds the 3 closest matches
- Uses majority voting to determine the recognized gesture
- Provides confidence level based on distance metrics

## Created By

Pavan Kalsariya (May 2025)

## License

This project is open source and available under the [MIT License](LICENSE).
