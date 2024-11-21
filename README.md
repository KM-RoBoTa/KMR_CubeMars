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
- Update full impedance example
- Write gravity compensation example (to test)
- check if 0 kept on a single power-up
