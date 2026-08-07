# PiCameraManager Component

F´ Prime component for Raspberry Pi camera image capture.

## Features

- Command-driven image capture via F´ Prime GDS
- libcamera integration for Raspberry Pi Camera Module v2
- Configurable image resolution (default 1920×1080)
- Event-based status reporting
- File management and downlink support

## Hardware Requirements

- Raspberry Pi Zero 2 W (or compatible Pi model)
- Pi Camera Module v2 (OV5647 sensor)
- CSI ribbon cable

## Software Dependencies

- F´ Prime v4.0.0+
- libcamera (Raspberry Pi camera stack)
- CMake 3.16+
- Python 3.8+ with fprime-tools

## Quick Start

### 1. Install Dependencies

```bash
# On Raspberry Pi
sudo apt-get install libcamera-apps libcamera-dev

# On development machine
pip install fprime-tools fprime-gds
```

### 2. Add to Your F´ Prime Project

Copy this component to your project:

```bash
cp -r PiCamera YourProject/Components/
```

### 3. Integrate with Topology

Add to your `topology.fpp`:

```fpp
instance piCameraMgr: Components.PiCameraManager base id 0x10005000

connections PiCameraCommands {
  cmdDisp.compCmdSend -> piCameraMgr.cmdIn
  piCameraMgr.cmdRegOut -> cmdDisp.compCmdReg
  piCameraMgr.cmdResponseOut -> cmdDisp.compCmdStat
}

connections PiCameraEvents {
  piCameraMgr.eventOut -> eventLogger.LogRecv
}

connections PiCameraTelemetry {
  piCameraMgr.tlmOut -> chanTlm.TlmRecv
}

connections PiCameraTime {
  piCameraMgr.timeCaller -> timeGetOut.timeGetPort
}
```

Add to your `CMakeLists.txt`:

```cmake
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../Components/PiCameraManager")
```

### 4. Build and Deploy

```bash
# Cross-compile for ARM
fprime-util generate aarch64-linux
fprime-util build aarch64-linux

# Deploy to Pi
scp build-artifacts/aarch64-linux/YourDeployment/bin/YourDeployment pi@<ip>:~/
```

### 5. Run and Test

```bash
# On Pi
./YourDeployment -a 0.0.0.0 -p 50000

# On development machine
fprime-gds --ip-address <pi-ip>
```

Then in GDS:
1. Navigate to Commanding tab
2. Send `piCameraMgr.ENABLE_CAMERA`
3. Send `piCameraMgr.CAPTURE_PHOTO`
4. Check Events tab for `PHOTO_CAPTURED_SUCCESS`

## Commands

| Command | Description |
|---------|-------------|
| `ENABLE_CAMERA` | Initialize camera subsystem |
| `CAPTURE_PHOTO` | Capture single image |
| `DOWNLOAD_IMAGE` | Download captured image via GDS |
| `DISABLE_CAMERA` | Shutdown camera subsystem |

## Events

| Event | Severity | Description |
|-------|----------|-------------|
| `CAMERA_ENABLED_EVENT` | ACTIVITY_LO | Camera enabled |
| `PHOTO_CAPTURED_SUCCESS` | ACTIVITY_HI | Image captured (includes path and size) |
| `PHOTO_CAPTURE_FAILED` | WARNING_HI | Capture failed (includes error) |

## Configuration

Default settings in component:
- **Image directory**: `/tmp/fprime_images/`
- **Resolution**: 1920×1080
- **Format**: JPEG
- **Filename**: `imgYYYYMMDD_HHMMSS.jpg`

## Documentation

- [Setup Guide](Components/PiCameraManager/docs/SETUP.md) - Detailed installation
- [Testing Guide](Components/PiCameraManager/docs/TESTING.md) - Validation procedures
- [Architecture](Components/PiCameraManager/docs/ARCHITECTURE.md) - Design documentation
- [API Reference](Components/PiCameraManager/docs/API.md) - Complete command and event details

## Troubleshooting

**Camera not detected**:
```bash
# Check I2C
sudo i2cdetect -y 10

# Test libcamera
libcamera-hello --list-cameras
```

**Build errors**:
```bash
# Install FPP tools
pip install fprime-fpp

# Clean and regenerate
fprime-util purge
fprime-util generate aarch64-linux
```

## Project Context

This component was developed as part of the first integration of NASA's F´ Prime flight software framework with the AMSAT CubeSatSim platform. 

**Complete integration** (AMSATFramer, RadioBridge, full deployment):  
https://github.com/maddywright3/fprime-amsat-reference

**Academic thesis**: "First Integration of NASA's F´ Prime Flight Software Framework with the AMSAT CubeSatSim Platform" - Madison Wright, Australian National University (2024)

## Testing

Component has been validated through:
- Hardware verification (I2C detection, libcamera functionality)
- Integration testing with F´ Prime GDS
- Image capture validation (format, resolution, integrity)
- Event logging verification
- File downlink capability testing

## Contributing

Contributions welcome! For major changes:
1. Open an issue to discuss the change
2. Fork the repository
3. Create a feature branch
4. Submit a pull request

## License

Apache 2.0 (following F´ Prime framework licensing)

## Author

**Madison Wright**  
Final Year Thesis Project  
Mechatronics & Computer Science  
Australian National University  
2024

## Acknowledgments

- NASA Jet Propulsion Laboratory for the F´ Prime framework
- fprime-community for the fprime-sensors repository
- AMSAT for the CubeSatSim platform
- Australian National University for project support

## Related Projects

- **F´ Prime Framework**: https://nasa.github.io/fprime/
- **fprime-sensors**: https://github.com/fprime-community/fprime-sensors
- **CubeSatSim**: https://github.com/alanbjohnston/CubeSatSim
- **Complete integration**: https://github.com/maddywright3/fprime-amsat-reference
