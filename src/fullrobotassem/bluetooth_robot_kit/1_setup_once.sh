#!/bin/bash
# Run ONCE per computer: installs packages and enables BlueZ compat mode.
set -e
echo "== Installing packages =="
sudo apt update
sudo apt install -y bluez python3-serial

echo "== Adding $USER to dialout group =="
sudo usermod -aG dialout "$USER"

echo "== Enabling BlueZ --compat mode =="
BTD=$(command -v bluetoothd || true)
[ -z "$BTD" ] && BTD=/usr/libexec/bluetooth/bluetoothd
[ -x /usr/libexec/bluetooth/bluetoothd ] && BTD=/usr/libexec/bluetooth/bluetoothd
sudo mkdir -p /etc/systemd/system/bluetooth.service.d
sudo tee /etc/systemd/system/bluetooth.service.d/compat.conf >/dev/null <<CONF
[Service]
ExecStart=
ExecStart=$BTD --compat
CONF
sudo systemctl daemon-reload
sudo systemctl restart bluetooth
rfkill unblock bluetooth || true

echo
echo "Done. LOG OUT AND BACK IN (or reboot) so the dialout group applies."
echo "Next: pair your phone once (see README.md, section 'Pair the phone')."
