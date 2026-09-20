#!/usr/bin/env python3
"""Verify a SIMCom modem and persist 115200 baud before PPP owns the UART."""

from __future__ import annotations

import argparse
import fcntl
import logging
import os
import pty
import select
import termios
import threading
import time


LOG = logging.getLogger("tripguard.modem")
BAUDS = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
}


def configure(fd: int, baud: int) -> None:
    if baud not in BAUDS:
        raise ValueError(f"unsupported baud: {baud}")
    attrs = termios.tcgetattr(fd)
    attrs[0] = termios.IGNPAR
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    attrs[4] = BAUDS[baud]
    attrs[5] = BAUDS[baud]
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)


def command(fd: int, text: str, timeout: float = 1.5) -> str:
    termios.tcflush(fd, termios.TCIFLUSH)
    os.write(fd, (text + "\r").encode("ascii"))
    deadline = time.monotonic() + timeout
    response = bytearray()
    while time.monotonic() < deadline:
        ready, _, _ = select.select(
            [fd], [], [], min(0.2, deadline - time.monotonic())
        )
        if not ready:
            continue
        chunk = os.read(fd, 1024)
        if chunk:
            response.extend(chunk)
            upper = bytes(response).upper()
            if b"\r\nOK\r\n" in upper or b"\nOK\r" in upper:
                break
            if b"ERROR" in upper:
                break
    return response.decode("ascii", errors="replace")


def responds(fd: int, baud: int, attempts: int = 3) -> bool:
    configure(fd, baud)
    for _ in range(attempts):
        if "OK" in command(fd, "AT").upper():
            return True
        time.sleep(0.1)
    return False


def prepare(port: str, target_baud: int) -> None:
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        detected = None
        for baud in (target_baud, 9600, 19200, 38400, 57600):
            if baud in BAUDS and responds(fd, baud):
                detected = baud
                break
        if detected is None:
            raise RuntimeError(f"modem did not return OK on {port}")
        LOG.info("modem responds at %d baud", detected)

        configure(fd, detected)
        if "OK" not in command(fd, f"AT+IPR={target_baud}", 2.0).upper():
            raise RuntimeError("AT+IPR did not return OK")
        time.sleep(0.25)
        configure(fd, target_baud)
        if "OK" not in command(fd, "AT&W", 2.0).upper():
            raise RuntimeError("AT&W did not return OK at target baud")

        for index in range(5):
            if "OK" not in command(fd, "AT", 1.5).upper():
                raise RuntimeError(
                    f"target-baud verification {index + 1}/5 failed"
                )
        LOG.info(
            "modem verified: five stable OK responses at %d baud", target_baud
        )
    finally:
        os.close(fd)


def self_test() -> None:
    master, slave = pty.openpty()
    slave_path = os.ttyname(slave)
    stop = threading.Event()

    def responder() -> None:
        pending = bytearray()
        while not stop.is_set():
            ready, _, _ = select.select([master], [], [], 0.1)
            if not ready:
                continue
            data = os.read(master, 1024)
            if not data:
                continue
            pending.extend(data)
            while b"\r" in pending:
                raw, _, pending = pending.partition(b"\r")
                if raw.strip():
                    os.write(master, b"\r\nOK\r\n")

    worker = threading.Thread(target=responder, daemon=True)
    worker.start()
    try:
        prepare(slave_path, 115200)
    finally:
        stop.set()
        worker.join(timeout=1.0)
        os.close(master)
        os.close(slave)
    print("tripguard_modem_prepare self-test: PASS")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "mode", nargs="?", choices=("prepare", "self-test"), default="prepare"
    )
    parser.add_argument("--port", default="/dev/ttyAMA3")
    parser.add_argument("--target-baud", type=int, default=115200)
    args = parser.parse_args()
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s"
    )
    if args.mode == "self-test":
        self_test()
    else:
        prepare(args.port, args.target_baud)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
