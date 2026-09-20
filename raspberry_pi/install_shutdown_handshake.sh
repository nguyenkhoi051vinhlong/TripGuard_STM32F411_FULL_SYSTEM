#!/bin/sh
set -eu

if [ "$(id -u)" -ne 0 ]; then
  printf '%s\n' 'Run this installer with sudo.' >&2
  exit 1
fi

install -d -m 0755 /opt/tripguard
install -m 0755 ./tripguard_shutdown_listener.py \
  /opt/tripguard/tripguard_shutdown_listener.py
install -m 0644 ./tripguard-shutdown.service \
  /etc/systemd/system/tripguard-shutdown.service

if [ -f /boot/firmware/config.txt ]; then
  CONFIG=/boot/firmware/config.txt
else
  CONFIG=/boot/config.txt
fi
OVERLAY='dtoverlay=gpio-poweroff,gpiopin=26,active_low=0'
if ! grep -Fqx "$OVERLAY" "$CONFIG"; then
  printf '\n%s\n' "$OVERLAY" >> "$CONFIG"
fi

systemctl daemon-reload
systemctl enable tripguard-shutdown.service
printf '%s\n' 'Installed and enabled, but not started.'
printf '%s\n' 'Verify BCM23 is LOW, then reboot before testing the handshake.'
