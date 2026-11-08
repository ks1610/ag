## Workspace Structure

```text
ag/
├── cad/
├── curcuit/
├── model/
│   ├── best.pt
│   ├── predict_webcam.py
│   ├── qr.png
│   ├── qrgenerator.py
│   ├── results.png
│   ├── train.py
│   └── yolov8n.pt
├── program/
│   ├── device/
│   │   ├── dongcodencoder/
│   │   │   ├── 1dongco.ino
│   │   │   └── dongcodencoder.ino
│   │   └── servo/
│   │       └── servo.ino
│   └── main/
│       ├── arduino1.ino
│       └── arduino2.ino
├── src/
│   └── fullrobotassem/
│       ├── bluetooth_robot_kit/
│       │   ├── 1_setup_once.sh
│       │   ├── 2_start_bluetooth.sh
│       │   └── 3_install_teleop.sh
│       ├── config/
│       │   ├── bridge.yaml
│       │   ├── joint_names_fullrobotassem.yaml
│       │   ├── mapper_params_online_async.yaml
│       │   └── nav2_params.yaml
│       ├── launch/
│       │   ├── display.launch
│       │   ├── display.launch.py
│       │   ├── gazebo.launch
│       │   ├── gazebo.launch.py
│       │   ├── mapping.launch.py
│       │   ├── maze_navigation.launch.py
│       │   ├── nav2_auto.launch.py
│       │   └── pick_and_place.launch.py
│       ├── maps/
│       │   ├── maze2_map.pgm
│       │   └── maze2_map.yaml
│       ├── meshes/
│       │   ├── base_link.STL
│       │   ├── left_arm.STL
│       │   ├── left_shoulder.STL
│       │   ├── left_wheel.STL
│       │   ├── right_arm.STL
│       │   ├── right_shoulder.STL
│       │   ├── right_wheel.STL
│       │   └── upper_body.STL
│       ├── scripts/
│       │   ├── bluetooth_teleop.py
│       │   ├── box_picker_node.py
│       │   ├── request_pick.py
│       │   └── upper_body_panel.py
│       ├── urdf/
│       │   ├── cardboard_box.sdf
│       │   ├── cardboard_box.urdf
│       │   ├── fullrobotassem.csv
│       │   ├── fullrobotassem.urdf
│       │   └── qr.png
│       ├── worlds/
│       │   ├── map1.sdf
│       │   ├── maze2.sdf
│       │   └── my_world.sdf
│       ├── .gitignore
│       ├── CMakeLists.txt
│       └── package.xml
├── .gitignore
├── CardboardBox.v1i.yolov8.zip
└── Readme.md
```



ros2 topic pub --once /pick_box_request std_msgs/msg/String "{data: '{\"x\": 2.875, \"y\": 0.530, \"yaw\": 2.795, \"qr_code\": \"mmb-6769\"}'}"

__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia glxinfo | grep -i "opengl renderer"
ros2 launch fullrobotassem pick_and_place.launch.py pick_x:=2.875 pick_y:=0.530 pick_yaw:=2.795

python3 request_pick.py --qr mmb-6769


# Real-robot bridge: Raspberry Pi → Arduino 1 (UART) → Arduino 2 (I2C)

Replaces, on real hardware, what `ros_gz_bridge` + `bridge.yaml` did in
Gazebo: it is the only piece that talks to actual motors/servos.

```
ROS2 (Pi)  --UART 115200--> Arduino 1 (master) --I2C 100kHz--> Arduino 2 (slave)
                                   |                                  |
                          PCA9685 (servos: arms)              BTS7960 x2 + JGA37 x2
                          BTS7960 #3 (lift) + 2 limit sw.          (wheels + encoders)
```

## Files
| File | Runs on | Purpose |
|---|---|---|
| `arduino1_master.ino` | Arduino 1 | UART bridge to the Pi, drives arm servos + lift motor, I2C master to Arduino 2 |
| `arduino2_slave.ino` | Arduino 2 | I2C slave, closed-loop wheel speed control + quadrature encoders |
| `hardware_bridge_node.py` | Raspberry Pi (ROS2) | Subscribes to `/cmd_vel`, `/left_arm/cmd_pos`, `/right_arm/cmd_pos`, `/upper_body/cmd_pos`; publishes `/odom`, `/joint_states`, limit-switch/lift status |

## Wiring (from `ketnoi.xlsx`)

**Arduino 1 (Master)**
| Component | Pin | Arduino 1 pin |
|---|---|---|
| PCA9685 | SDA / SCL | A4 / A5 |
| BTS7960 #3 (lift) | EN | 5V (always on) |
| BTS7960 #3 (lift) | RPWM / LPWM | D5 / D6 |
| Button #1 (upper limit) | OUT | D2 |
| Button #2 (lower limit) | OUT | D3 |
| Arduino 2 | SDA / SCL | A4 / A5 (shared bus) |

**Arduino 2 (Slave)**
| Component | Pin | Arduino 2 pin |
|---|---|---|
| BTS7960 #1 (left wheel) | EN / RPWM / LPWM | D5 / D4 / D6 |
| BTS7960 #2 (right wheel) | EN / RPWM / LPWM | D11 / D12 / D10 |
| JGA37 #1 (left) encoder | Ch A / Ch B | D2 (INT0) / D7 |
| JGA37 #2 (right) encoder | Ch A / Ch B | D3 (INT1) / D8 |
| Arduino 1 | SDA / SCL | A4 / A5 (shared bus) |

**Pi ↔ Arduino 1 (UART)** — not in the spreadsheet, pick any free UART:
- Pi TX → Arduino 1 RX (D0): safe to wire directly.
- **Arduino 1 TX (D1) → Pi RX: MUST go through a level shifter / resistor
  divider** — the Pi's GPIO is 3.3V-only and Arduino runs at 5V.
- Common ground between Pi and both Arduinos.

## Protocols
See the header comments in `arduino1_master.ino` (UART ASCII lines) and
`arduino2_slave.ino` (I2C byte format) for the exact message layout.

## Things you must calibrate before trusting the odometry / lift speed
- `TICKS_PER_REV` in `arduino2_slave.ino` — depends on your JGA37 gearbox
  and how it's wired (channel-A-only, both edges). Spin one wheel exactly
  one revolution by hand and count ticks to get the real value, then
  mirror the same number in `hardware_bridge_node.py`'s `_update_odometry`.
- `KP` / `KI` in `arduino2_slave.ino` — start low, raise until the wheel
  tracks a step speed command without much overshoot.
- `LIFT_PWM_SPEED` / `LIFT_TIMEOUT_MS` in `arduino1_master.ino` — set the
  timeout comfortably longer than the real travel time between limit
  switches.

## Deploying
1. Flash `arduino1_master.ino` to Arduino 1 and `arduino2_slave.ino` to
   Arduino 2 (Arduino IDE; `arduino1_master.ino` needs the *Adafruit PWM
   Servo Driver Library* installed).
2. Wire everything per the tables above.
3. On the Pi, install `pyserial` (`pip install pyserial`) and add
   `hardware_bridge_node.py` as a `Node` in your real-robot launch file —
   in place of the `ros_gz_sim`/bridge nodes that `pick_and_place.launch.py`
   uses for simulation. `box_picker_node.py` and `request_pick.py` need no
   changes: they already publish/subscribe the exact topics this bridge
   speaks.