#!/usr/bin/env python3
"""
test_forward_backward.py
------------------------------------------------------------------
Bench test — NO Raspberry Pi / ROS2 required.

Talks directly to Arduino 1 over USB-serial using the exact same ASCII
protocol hardware_bridge_node.py will use later:

    PC --USB serial--> Arduino 1 (master) --I2C--> Arduino 2 (slave)

Sends "V <lin_mm_s> <ang_mrad_s>" to drive both wheels straight, reads
back Arduino 1's "O ..." telemetry line (which carries the encoder ticks
Arduino 2 reports over I2C), and stops each phase once the wheels have
turned the requested number of revolutions.

Usage:
    python test_forward_backward.py --port COM5
    python test_forward_backward.py --port COM5 --rounds 5 --speed 150
    python test_forward_backward.py --port /dev/ttyUSB0   (Linux/Mac)

Install dependency once:
    pip install pyserial
"""

import argparse
import sys
import time

import serial


def parse_args():
    p = argparse.ArgumentParser(description='Forward/backward encoder-based bench test.')
    p.add_argument('--port', required=True, help='Serial port Arduino 1 is on, e.g. COM5 or /dev/ttyUSB0')
    p.add_argument('--baud', type=int, default=115200)
    p.add_argument('--speed', type=int, default=150, help='Commanded linear speed in mm/s')
    p.add_argument('--rounds', type=float, default=5.0, help='Number of wheel revolutions per phase')
    p.add_argument('--ticks-per-rev', type=int, default=990,
                    help='Must match TICKS/PULSES_PER_REV used on Arduino 2 (default from your test sketch)')
    p.add_argument('--timeout-s', type=float, default=30.0, help='Safety timeout per phase')
    return p.parse_args()


def send_line(ser, text):
    ser.write((text + '\n').encode('ascii'))
    print(f'  -> sent: {text}')


def run_phase(ser, label, lin_mm_s, target_ticks, timeout_s):
    """
    Commands a wheel speed and waits until the average absolute encoder
    tick count (relayed from Arduino 2 through Arduino 1's telemetry)
    reaches target_ticks, then stops.
    """
    print(f'\n=== Phase: {label} ===')
    accum_left = 0
    accum_right = 0
    deadline = time.time() + timeout_s

    send_line(ser, f'V {lin_mm_s} 0')

    while time.time() < deadline:
        raw = ser.readline().decode('ascii', errors='ignore').strip()
        if not raw:
            continue

        if raw[0] != 'O':
            # Likely boot banner or an unrelated line — print and ignore.
            print(f'  (ignored) {raw}')
            continue

        parts = raw.split()
        if len(parts) != 7:
            continue

        try:
            dt_ms, left_ticks, right_ticks, btn_u, btn_l, lift_state = (
                int(parts[1]), int(parts[2]), int(parts[3]),
                int(parts[4]), int(parts[5]), int(parts[6])
            )
        except ValueError:
            continue

        accum_left += abs(left_ticks)
        accum_right += abs(right_ticks)
        avg_ticks = (accum_left + accum_right) / 2.0

        print(f'  left={accum_left:6d} right={accum_right:6d} '
              f'avg={avg_ticks:7.1f}/{target_ticks:.0f} ticks  '
              f'btn_upper={btn_u} btn_lower={btn_l} lift={lift_state}')

        if avg_ticks >= target_ticks:
            send_line(ser, 'V 0 0')
            print(f'  -> target reached, phase "{label}" complete.')
            time.sleep(0.3)  # let it actually stop before the next phase
            return True

    # Timed out without reaching target — stop for safety and report.
    send_line(ser, 'V 0 0')
    print(f'  !! timeout waiting for "{label}" to complete (safety stop sent).')
    return False


def main():
    args = parse_args()
    target_ticks = args.rounds * args.ticks_per_rev

    print(f'Opening {args.port} @ {args.baud} baud...')
    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    time.sleep(2.0)  # allow Arduino to finish its reset after DTR toggle
    ser.reset_input_buffer()

    try:
        ok_fwd = run_phase(ser, 'FORWARD', args.speed, target_ticks, args.timeout_s)
        time.sleep(1.0)
        ok_bwd = run_phase(ser, 'BACKWARD', -args.speed, target_ticks, args.timeout_s)

        print('\n=== Summary ===')
        print(f'Forward phase:  {"OK" if ok_fwd else "TIMED OUT"}')
        print(f'Backward phase: {"OK" if ok_bwd else "TIMED OUT"}')

    except KeyboardInterrupt:
        print('\nInterrupted by user.')
    finally:
        send_line(ser, 'V 0 0')  # always leave the robot stopped
        ser.close()


if __name__ == '__main__':
    main()