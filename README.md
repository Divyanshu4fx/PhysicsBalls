# PhysicsBalls

A real-time physics simulation running on an ESP32 microcontroller that displays particle physics affected by real-world gravity measured through an accelerometer. The simulation features 500 particles that respond to the physical orientation of the device, creating an interactive and engaging visual experience.

## Features

- Real-time physics simulation with 500 particles
- Gravity control through ADXL345 accelerometer
- Double-buffered display for smooth animations
- Multi-core task management for optimal performance
- ~30 FPS display refresh rate
- Collision detection and response between particles
- Boundary collision handling with velocity dampening
- Hardware-optimized SPI communication

## Hardware Requirements

- ESP32 development board
- ST7735 TFT Display (128x160 resolution)
- ADXL345 Accelerometer
- Connecting wires

### Pin Connections

#### TFT Display
- SCLK: GPIO 14
- MOSI: GPIO 13
- RST: GPIO 12
- DC: GPIO 2
- CS: GPIO 15

#### ADXL345 Accelerometer
- Connect to ESP32's I2C pins (default SDA/SCL)
- VCC: 3.3V
- GND: Ground

## Software Dependencies

The following Arduino libraries are required:

```
- Adafruit_GFX
- Adafruit_ST7735
- Adafruit_Sensor
- Adafruit_ADXL345_U
- Wire
- SPI
```

## Installation

1. Install the Arduino IDE
2. Add ESP32 board support to Arduino IDE
3. Install all required libraries through the Arduino Library Manager
4. Clone this repository
5. Open `Sensor_Integrated.ino` in Arduino IDE
6. Select your ESP32 board from Tools > Board menu
7. Select the correct port from Tools > Port menu
8. Upload the sketch to your ESP32

## How It Works

### Physics Simulation
- Particles are simulated with position, velocity, and mass properties
- Collision detection uses a simple distance-based approach
- Elastic collisions are calculated with conservation of momentum
- Boundary collisions include velocity dampening for realistic behavior

### Task Management
The program runs three main tasks distributed across both ESP32 cores:

1. **Physics Task** (Core 0)
   - Updates particle positions and velocities
   - Handles particle-particle collisions
   - Runs at ~30Hz

2. **Display Task** (Core 1)
   - Manages double-buffered display updates
   - Draws particles to screen
   - Synchronized with vsync for smooth animation

3. **Sensor Task** (Core 0)
   - Reads accelerometer data
   - Updates gravity vector
   - Runs at 100Hz

### Performance Optimizations
- Double buffering to prevent screen tearing
- Optimized SPI settings for fast display updates
- Efficient memory usage with static frame buffers
- Task prioritization for smooth physics calculations
- Mutex-protected shared resource access

## Configuration

Key parameters can be adjusted in the code:

```cpp
const int numBalls = 500;          // Number of particles
const float restitution = 0.20;    // Collision bounciness
const float GRAVITY_SCALE = 1;     // Gravity sensitivity
const int vsyncPeriod = 33;        // Frame timing (ms)
```

## Acknowledgments

- Adafruit for their excellent display and sensor libraries
- ESP32 community for multi-core programming examples
- Physics simulation inspired by particle system implementations

## Troubleshooting

### Common Issues

1. **Display Not Working**
   - Check SPI connections
   - Verify TFT display model and initialization parameters
   - Ensure proper power supply voltage

2. **Accelerometer Not Detected**
   - Verify I2C connections
   - Check accelerometer address if modified
   - Confirm proper voltage levels

3. **Poor Performance**
   - Reduce number of particles
   - Check CPU frequency settings
   - Monitor task stack sizes

## Future Improvements

- Add different particle types with varying properties
- Implement particle color based on velocity
- Add touch screen interaction support
- Create configurable simulation parameters
- Add wireless control capabilities
