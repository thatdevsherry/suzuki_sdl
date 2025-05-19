# Suzuki SDL protocol scanner

Scanner that I made through several procedures, including:

- scouring through forums to understand SDL structure (credit to Rhinopower)
- passively listening on comms b/w ECU and expensive AUTEL scanner to get (obd address <-> parameter) mapping
- simulating an ECU (`sim.py`) and connecting to AUTEL scanner to figure out calculations by sending a couple of fixed values
for each address to derive an accurate formula

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

See it [in action](https://youtu.be/AQNUMpRlyos).
