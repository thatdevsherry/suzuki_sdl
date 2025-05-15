# Suzuki SDL protocol scanner

Scanner that made by simulating an ECU and connecting to an AUTEL scanner for getting calculations.

NOTE: It is tested on and assumes connecting to a Suzuki Baleno with G13BB engine and ECU (33920-65GP).
Some parameters will be wrong when using on another ECU.

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
