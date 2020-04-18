#!/bin/python3
#
# Example of a pressure profile

import argparse
import time
from alakol_datalogger import AlakolSocket
from alakol_command import AlakolCommand

# ------------------------------ main ------------------------------

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("ip_addr", nargs="?",
                        default="192.168.1.77")
    args = parser.parse_args()

    # Command
    alakol = AlakolCommand(args.ip_addr)
    alakol.command(80, True)  # Inflate at 80%

    # Stream data
    streaming = AlakolSocket()
    streaming.connect(args.ip_addr, 8080)
    print(f"Connected to {args.ip_addr}:8080 !")

    for idx, data in enumerate(streaming.cbor_receive()):
        gauge_pressure = data["gauge_pressure"]
        print(gauge_pressure)

        if gauge_pressure > 20:
            alakol.command(0, True)  # Hold
            time.sleep(3)
            alakol.command()  # Deflate
