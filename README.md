# Suzuki SDL protocol scanner

Scanner that made by simulating an ECU and connecting to an AUTEL scanner for getting calculations.

## Env setup

```bash
pip install pip-tools
pip-compile && pip-sync
```

## Running

A basic command to run and see raw values

```bash
python ./main.py --serial-port /dev/ttyUSB0 --timeout=30 --table RAW
```
