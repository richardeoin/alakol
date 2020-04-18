_Alakol_ software
======

A selection of scripts for capturing data / commanding _alakol_. Data is
streamed over TCP port 8080 in [CBOR](https://cbor.io/) packed format.

The control interface is a HTTP server on TCP port 80.

# Getting Started

These are python3 scripts, it's a good idea to run them in an virtual
enviroment (venv). You can set this up and install dependancies like so:

```
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

There's also a jupyter notebook for plotting streamed data live.

## On Windows

Maybe this works under WSL?

# Examples

See [/profile_example.py][]

# Usage

The `AlakolSocket` class provides a method `cbor_receive` that yields each
datapoint.

```python3
from alakol_datalogger import AlakolSocket

# Connect
streaming = AlakolSocket()
streaming.connect("192.168.x.x", 8080)

# Iterate over received datapoints
for idx, data in enumerate(streaming.cbor_receive()):
    gauge_pressure = data["gauge_pressure"]
    print(gauge_pressure)
```

Commanding is even easier. Under the hood this just uses [requests](https://requests.readthedocs.io/).

```python3
from alakol_command import AlakolCommand

# Connect
alakol = AlakolCommand("192.168.x.x")

# Set motor state (percentage) and value (True = close, False = open)
alakol.command(80, True)     # Inflate at 80%
```

# License

[MIT](LICENSE.md)
