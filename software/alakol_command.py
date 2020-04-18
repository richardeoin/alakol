#!/bin/python3
#
# Alakol Datalogger!

import requests
import argparse


class AlakolCommand:
    """Commands the motor and valve on the Alakol Datalogger, via the its
    HTTP interface
    """

    def __init__(self, address, port=80):
        self.url = f"http://{address}:{port}/"

    def command(self, motor_speed_percent=0, close_valve=False):
        if (
            type(motor_speed_percent) is not int
            or motor_speed_percent < 0
            or motor_speed_percent > 100
        ):
            raise (Exception("Invalid motor speed percentage"))

        requests.post(
            f"{self.url}command",
            data={
                "motor-speed": str(motor_speed_percent),
                "valve-state": "closed" if close_valve else "open",
            },
            headers={"Content-Type": "application/x-www-form-urlencoded"},
        )


# ------------------------------ main ------------------------------

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("ip_addr", nargs="?", default="192.168.1.77")
    parser.add_argument(
        "-m",
        "--motor",
        dest="motor",
        default=0,
        help="Motor speed percentage (default = 0%)",
    )
    parser.add_argument(
        "-c",
        "--close-valve",
        dest="valve",
        action="store_true",
        help="Close the air valve (default = open)",
    )
    args = parser.parse_args()

    alakol = AlakolCommand(args.ip_addr)
    alakol.command(int(args.motor), args.valve)
