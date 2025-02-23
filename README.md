
# Anemometer Project

This project is a refactored, industrial-grade firmware for an ESP32-based anemometer. The code has been reorganized using a class-based, object-oriented approach in C++ to improve modularity, maintainability, and readability—all while preserving the original logic.

## Project Structure

```
Anemometer/
├── lib
│   ├── AxisControl
│   │   ├── AxisControl.h
│   │   └── AxisControl.cpp
│   ├── BluetoothApp
│   │   ├── BluetoothHandler.h
│   │   └── BluetoothHandler.cpp
│   ├── BT_String
│   │   └── BluetoothCommand.h
│   ├── Button
│   │   ├── Debounce.h
│   │   └── Debounce.cpp
│   ├── Buzzer
│   │   ├── Buzzer.h
│   │   └── Buzzer.cpp
│   ├── EEPROM
│   │   ├── ROM.h
│   │   └── ROM.cpp
│   ├── LEDRing
│   │   ├── LEDRing.h
│   │   └── LEDRing.cpp
│   ├── LineFollow
│   │   ├── LineFollow.h
│   │   └── LineFollow.cpp
│   ├── Motor
│   │   ├── Motor.h
│   │   └── Motor.cpp
│   ├── PinDef
│   │   ├── PinDef.h
│   │   └── PinDef.cpp
│   ├── SystemVars
│   │   ├── SystemVars.h
│   │   └── SystemVars.cpp
│   └── StateMachine
│       ├── RobotController.h
│       └── RobotController.cpp
└── src
    └── main.cpp
```

## Features

- **Modular Design:** Each hardware and logical component is encapsulated within its own class.
- **State Machine:** Implements a robust state machine for calibration, line following, axis control, and more.
- **Bluetooth Communication:** Supports commands via Bluetooth using the ESP32's BluetoothSerial.
- **Visual Feedback:** Uses LED patterns for status indication and debugging.
- **EEPROM Integration:** Manages persistent calibration data using the Preferences library.
- **Motor Control:** Utilizes the BTS7960 driver library for robust motor control.

## Getting Started

### Requirements

- **Hardware:** ESP32 (esp32doit-devkit-v1 recommended)  
- **Libraries:** 
  - Arduino framework for ESP32  
  - [BTS7960](https://github.com/your-library-url)  
  - [FastLED](http://fastled.io/)  
  - [QTRSensors](https://www.pololu.com/docs/0J18)  
  - Preferences (included with ESP32 Arduino framework)

### Setup and Build

1. **Clone the Repository:**

   ```bash
   git clone https://github.com/austin207/Class-based-Anemometer-Architecture.git
   cd Anemometer
   ```

2. **Build and Upload:**

   Use PlatformIO or your preferred IDE to build and upload the firmware to your ESP32.

   For PlatformIO:

   ```bash
   platformio run --target upload
   ```

## Usage

Once uploaded, the firmware will:

- Perform sensor calibration and line following.
- Control motor actions based on the state machine implemented in `RobotController.cpp`.
- Allow control via physical buttons or over Bluetooth with commands like `GO` and `CALIBRATE`.

## Contributing

Contributions are welcome! Please fork the repository and submit pull requests with any improvements or bug fixes.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

### Final Steps

1. **Save the above content into a file named `README.md`** in your project’s root folder.
2. **Add, commit, and push the file to your repository:**

   ```bash
   git add README.md
   git commit -m "Add improved README file"
   git push
   ```
