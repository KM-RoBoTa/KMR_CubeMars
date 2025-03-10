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
- Write gravity compensation example (to test)
- check if 0 kept on a single power-up

- Check if no need to maintain position when 0-setting now that we stop the motors
- Check if Kp, Kd can be varied during the run (+check if compatible on-the-run with the lib)
- Check if the full impedance control actually works (use the software for easier checking)