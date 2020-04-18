#!/bin/python3
#
# Alakol Datalogger!

import socket
from cbor2 import loads
import argparse
import os
from time import gmtime, strftime, time


class AlakolSocket:
    """Read data from Alakol Datalogger, via a TCP socket
    """

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def connect(self, host, port):
        self.sock.connect((host, port))

    def cbor_receive(self):
        while True:
            chunk = self.sock.recv(2048)
            if chunk == b"":
                raise RuntimeError("socket connection broken")
            # Try to decode as CBOR
            try:
                cbor_data = loads(chunk)
                yield cbor_data.value
            except:
                None

    def disconnect(self):
        self.sock.disconnect()


# ------------------------------ main ------------------------------

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("ip_addr", nargs="?", default="192.168.1.77")
    parser.add_argument(
        "-n",
        "--no-save",
        dest="no_save",
        action="store_true",
        help="Don't save the data to file",
    )
    parser.add_argument(
        "-s",
        "--silent",
        dest="silent",
        action="store_true",
        help="Don't print whilst receving data",
    )
    parser.add_argument(
        "-d",
        "--decimate",
        dest="n",
        default="1",
        help="Record only every Nth sample, drop the rest",
    )
    parser.add_argument(
        "-f",
        "--filename",
        dest="filename",
        default="alakol_datalogger",
        help="Set the beginning of the filename",
    )
    args = parser.parse_args()

    # Decimation
    try:
        decimation = int(args.n)
    except:
        print("Failed to set decimation correctly!")
        decimation = 1

    # Output files
    time_string = strftime("%y%m%d_%H%M%S", gmtime())
    csv_filename = f"{args.filename}_{time_string}.csv"

    with open(csv_filename, "w+") as csv_fp:
        wrote_header = False

        # Stream data
        streaming = AlakolSocket()
        streaming.connect(args.ip_addr, 8080)
        print(f"Connected to {args.ip_addr}:8080 !")

        for idx, data in enumerate(streaming.cbor_receive()):
            if (idx % decimation) == 0:
                record = {"timestamp": time()}
                record.update(data)

                if not args.silent:
                    print(record)

                if not args.no_save:
                    # Write header to CSV if required
                    if not wrote_header:
                        keys = [key for key in record]
                        csv_fp.write(",".join(keys) + os.linesep)
                        wrote_header = True

                    # Write record to CSV
                    values = [
                        f"{record[key]:.3f}" if record[key] else "" for key in record
                    ]
                    csv_fp.write(", ".join(values) + os.linesep)
                    csv_fp.flush()
