# OSVR-ART plugin

## Usage instructions

Before loading the plugin, open DTrack2 software:

1. Make sure that one or more bodies are calibrated and tracked
2. Output settings: Send to -> this computer. Port 5000 (make sure that your system doesn't use or block this port)

Make a note of the IP address of the tracking system and specify it in `serverHost` parameter in server config and specify port (if different from 5000) in `dataPort` parameter. By default plugin will use `192.168.0.1` and port `5000`.

The ART system supports a varying number of tracked devices. Please refer the ART documentation to determine the number of max tracked devices for your license. It supports at least 4 tracked objects (including Flysticks and Fingertracking hand devices).

The device descriptor accounts for 4 tracked bodies and 4 tracked Flysticks.
Depending on your setup, the plugin will automatically recognize which device are tracked (reported by ART system), and send reports as they're available.

## Building

Copy the `include` and `src` directories from the DTrack SDK into the `DTrackSDK` directory before running CMake.
