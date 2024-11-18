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

## Todo
- Specific feedbacks?
- Motor pinging
- Rename functions
- Automatic stopping the motors when disabling them
- Update full impedance example
- Write gravity compensation example
