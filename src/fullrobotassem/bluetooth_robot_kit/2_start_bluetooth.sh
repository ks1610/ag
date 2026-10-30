#!/bin/bash
# Run EVERY TIME before using the robot (keep this terminal open).
# Registers the serial service and waits for the phone to connect.
# Press Ctrl+C to stop.

[ "$EUID" -ne 0 ] && exec sudo "$0" "$@"

ADAPTER=${1:-hci0}
CHANNEL=1

rfkill unblock bluetooth
hciconfig "$ADAPTER" up 2>/dev/null
bluetoothctl <<BT >/dev/null
power on
discoverable on
pairable on
BT

# SDP socket permission resets on reboot
chmod 777 /var/run/sdp 2>/dev/null
if ! sdptool browse local 2>/dev/null | grep -q "Serial Port"; then
  sdptool add --channel=$CHANNEL SP
fi

# Free any stale binding
rfcomm release all 2>/dev/null

# Keep /dev/rfcomm0 readable by your user every time the phone (re)connects
(
  while true; do
    [ -e /dev/rfcomm0 ] && chmod 666 /dev/rfcomm0
    sleep 1
  done
) &
FIXER=$!
trap 'kill $FIXER 2>/dev/null; rfcomm release all 2>/dev/null; exit 0' INT TERM EXIT

echo "=============================================="
echo " Waiting for phone on $ADAPTER channel $CHANNEL"
echo " Open the app and connect to this computer."
echo " Then start ROS in another terminal:"
echo "   ros2 launch fullrobotassem mapping.launch.py"
echo "=============================================="

# One connection per loop; release stale bindings between connections
while true; do
  rfcomm release all 2>/dev/null
  rfcomm listen "$ADAPTER" $CHANNEL
  echo "Phone disconnected. Listening again in 1 s..."
  sleep 1
done
