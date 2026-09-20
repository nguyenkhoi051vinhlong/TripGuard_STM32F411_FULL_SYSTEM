#!/usr/bin/env python3
"""Request a clean Raspberry Pi shutdown when STM32 asserts BCM GPIO23."""

from signal import pause
from subprocess import run
from threading import Lock

from gpiozero import Button


REQUEST_GPIO = 23
REQUEST_HOLD_SECONDS = 1.0
_shutdown_lock = Lock()


def request_poweroff() -> None:
    if not _shutdown_lock.acquire(blocking=False):
        return
    # The gpio-poweroff overlay drives BCM26 only when the kernel reaches its
    # power-off path. STM32 treats that signal, not this userspace callback,
    # as PI_SHUTDOWN_ACK.
    run(("/usr/bin/systemctl", "poweroff"), check=False)


shutdown_request = Button(
    REQUEST_GPIO,
    pull_up=False,
    bounce_time=0.05,
    hold_time=REQUEST_HOLD_SECONDS,
    hold_repeat=False,
)
# A one-second qualified HIGH rejects boot glitches and short EMI pulses.
shutdown_request.when_held = request_poweroff
pause()
