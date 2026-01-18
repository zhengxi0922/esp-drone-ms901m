# ESP-Drone UDP Controller (PC GUI)

This tool sends CRTP setpoint packets over Wi-Fi UDP to the drone.

## Requirements
- Python 3 (with Tkinter)

## How to use
1) Connect your PC to the drone AP (default SSID `ESP-DRONE_xxx`).
2) Run:

```bash
python python_controller/udp_controller.py
```

3) Keep IP `192.168.43.42` and port `2390` (default).
4) Click `Ping` to verify the UDP link (CFPING/CFPONG, should show `Link: OK`).
5) Click `Unlock (thrust=0)` once.
6) Set thrust >= 1000 and click `Start Stream`.

## Flight GUI (takeoff + forward/back + telemetry)
1) Connect your PC to the drone AP.
2) Run:

```bash
python python_controller/flight_gui.py
```

3) Click `Ping`, then `Start` under Telemetry.
4) Click `Unlock (thrust=0)` once.
5) Use `Takeoff`, `Forward`, `Backward`, `Land`.

## Notes
- The controller sends CRTP setpoint packets: header `0x30` + `<roll, pitch, yaw, thrust>` + checksum.
- For safety, remove props during testing.
