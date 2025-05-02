# CubeMars C++ lib for AK motors

## Setup
Get IP:
```bash
sudo arp-scan --localnet
```

Set up can:
```bash
sudo ip link set can0 up type can bitrate 1000000
```

## Quick notes
- Need RTOS to get rid of delays
- When moving, sending reading command does not work, need to resend previous command to get up-to-date fbck
- Atm, send current position command for zero-setting

## Todo
- Update full impedance example
- Update + explain PID tuning in examples + mention typical Kp, Kd values without load
- Talk about the PREEMPT multithreading (+ expected behavior in examples without overtimes)
- Write github version
- Add set up CAN in both versions of the documentation