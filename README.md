# ros2_pmw3901_driver
**ROS 2 driver for the PMW3901 optical flow sensor**.  
This package publishes motion counts (`delta_x`, `delta_y`) and optionally the frame buffer as ROS 2 topics.

It is useful for robotics applications such as drone navigation, odometry, and velocity estimation.

---

## Features
- Reads motion counts (`Δx`, `Δy`) from the PMW3901 sensor
- Publishes motion and frame data as ROS 2 topics
- Frame buffer access for debugging and visualization
- Written in C++ with ROS 2 conventions

---

## Installation
Clone into your ROS 2 workspace:

- `cd ~/ros2_ws/src`
- `git clone https://github.com/Hohyeong/ros2_pmw3901_driver.git`
- `cd ..`
- `colcon build`

---

## Hardware Setup (Raspberry Pi 5)

This package has been tested on **Raspberry Pi 5**.

To use the PMW3901 sensor via SPI, connect the following pins:

| PMW3901 Pin | Raspberry Pi 5 Pin |
|-------------|--------------------|
| **GND**     | GND                |
| **VCC**     | 3.3V               |
| **MISO**    | SPI0 MISO (GPIO 9, Pin 21) |
| **MOSI**    | SPI0 MOSI (GPIO 10, Pin 19) |
| **SCLK**    | SPI0 SCLK (GPIO 11, Pin 23) |
| **CS**      | GPIO 8 (Pin 24) |
| **MOTION (INT)** | GPIO 19 (Pin 35) |

Make sure that:
- The SPI interface is enabled (`raspi-config` → Interfaces → Enable SPI).  
- CS (chip select) is connected to **BCM 8**.  
- MOTION (interrupt) pin is connected to **BCM 19**.

---

## Usage
- `ros2 run pmw3901_ros2 pmw3901_motion`
- `ros2 run pmw3901_ros2 pmw3901_frame_grab`


---

## License
This project is licensed under the Apache2.0 License.

---

### Third-party code
This project includes portions of code from the following repositories:
- https://github.com/pimoroni/pmw3901-python/tree/main (MIT License)
- https://github.com/bitcraze/Bitcraze_PMW3901/tree/master (MIT License)

The original license texts are provided in the `LICENSES` directory.
