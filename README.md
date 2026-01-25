# Robocar

Autonomous RC car project with navigation capabilities using LiDAR and RTK GPS.

## Hardware

- **Platform**: RC Car with Ackermann steering
- **Computer**: NVIDIA Jetson Nano
- **LiDAR**: LD19 360° 2D LiDAR
- **GPS**: Point One RTK GPS (centimeter accuracy)
- **Camera**: OAK-D DepthAI stereo camera
- **Motor Controller**: VESC ESC

## Project Structure

```
Robocar/
├── src/
│   ├── drivers/           # Hardware drivers (LiDAR, GPS, VESC)
│   ├── perception/        # Sensor processing & fusion
│   ├── slam/              # SLAM & localization
│   ├── navigation/        # Path planning & following
│   ├── control/           # Low-level control
│   ├── core/              # Configuration, state machine
│   └── visualization/     # Debug & monitoring tools
├── config/                # YAML configuration files
├── scripts/               # Test & run scripts
└── docs/                  # Documentation
```

## Quick Start

```bash
# Install dependencies
pip install -r requirements.txt

# Test sensors
python scripts/test_lidar.py /dev/ttyUSB0
python scripts/test_gps.py /dev/ttyUSB1
python scripts/test_all_sensors.py
```

## Documentation

See `docs/PROJECT_PLAN.md` for the complete project roadmap and technical details.

## Current Status

- [x] Simulation environment (ML-Agents)
- [x] Camera segmentation (SegNet)
- [x] Gamepad control
- [x] LiDAR driver (LD19)
- [x] GPS driver (Point One RTK)
- [ ] SLAM implementation
- [ ] Autonomous navigation
