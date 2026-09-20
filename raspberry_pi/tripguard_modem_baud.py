#!/usr/bin/env python3
"""Set the TripGuard UART modem to a fixed, persistent baud rate."""

import sys
import time

import serial


PORT = "/dev/ttyAMA3"
CURRENT_BAUD = 9600
TARGET_BAUD = 115200


def exchange(baud, command, wait_seconds=1.0):
    with serial.Serial(PORT, baud, timeout=2) as modem:
        modem.reset_input_buffer()
        modem.write((command + "\r\n").encode("ascii"))
        modem.flush()
        time.sleep(wait_seconds)
        return modem.read(512).decode("ascii", errors="replace")


def main():
    response = exchange(CURRENT_BAUD, "AT")
    if "OK" not in response:
        print("ERROR: modem did not answer AT at 9600")
        return 1
    print("AT at 9600: OK")

    query = exchange(CURRENT_BAUD, "AT+IPR?")
    print("Current IPR response:", repr(query))

    change = exchange(CURRENT_BAUD, "AT+IPR=115200", wait_seconds=1.5)
    print("Set IPR response:", repr(change))
    time.sleep(1.0)

    response = exchange(TARGET_BAUD, "AT")
    if "OK" not in response:
        print("ERROR: modem did not answer AT at 115200 after AT+IPR")
        return 2
    print("AT at 115200: OK")

    saved = exchange(TARGET_BAUD, "AT&W")
    if "OK" not in saved:
        print("WARNING: AT&W did not return OK:", repr(saved))
    else:
        print("Profile saved: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
