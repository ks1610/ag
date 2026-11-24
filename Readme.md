# fullrobotassem (ag)

ROS 2 Jazzy package for a differential-drive robot with arms, lift, and a 2D lidar. It runs in Gazebo simulation and, with the hardware bridge, on a Raspberry Pi 4 with an LD14 lidar.

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

## Requirements

- Ubuntu 24.04 (Noble) on x86_64 for simulation, or Raspberry Pi 4 (64-bit Ubuntu 24.04) for the real robot
- ROS 2 Jazzy
- Gazebo Harmonic (installed through `ros-jazzy-ros-gz`) for simulation
- Python 3.12 (the Ubuntu 24.04 default)

## 1. Install ROS 2 Jazzy

```bash
sudo apt update && sudo apt install -y locales software-properties-common curl git
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source /opt/ros/jazzy/setup.bash
```

Use `ros-jazzy-ros-base` instead of `ros-jazzy-desktop` on a Raspberry Pi, where RViz can run on another computer.

## 2. Install the ROS packages used by this project

```bash
sudo apt install -y \
  ros-jazzy-ros-gz ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui ros-jazzy-xacro \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-nav2-map-server \
  python3-serial
```

`python3-serial` comes from apt because Ubuntu 24.04 blocks system-wide `pip install`.

## 3. Clone and build

The repo tracks generated `build/`, `install/`, and `log/` folders from another computer. Remove them before the first build so CMake doesn't reuse paths from that machine.

```bash
cd ~
git clone https://github.com/ks1610/ag.git
cd ~/ag
rm -rf build install log

rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source ~/ag/install/setup.bash
```

Add the source line to `~/.bashrc` if you use this workspace often:

```bash
echo "source ~/ag/install/setup.bash" >> ~/.bashrc
```

If `rosdep install` reports a missing key, install the named package with apt and rerun the build.

---

# PART A: Simulation (Gazebo, no real hardware)

Open a new terminal for each step, and source the workspace in each one (`source ~/ag/install/setup.bash`).

**Check the arguments of each launch file first.** This prints the arguments a launch file accepts, so you don't have to read the source:

```bash
ros2 launch fullrobotassem gazebo.launch.py --show-args
ros2 launch fullrobotassem pick_and_place.launch.py --show-args
```

Repeat for `mapping.launch.py`, `nav2_auto.launch.py`, and `maze_navigation.launch.py` as needed.

**Robot model only (no Gazebo):**

```bash
ros2 launch fullrobotassem display.launch.py
```

This starts `robot_state_publisher`, `joint_state_publisher_gui` (joint sliders), and RViz.

**Gazebo simulation:**

```bash
ros2 launch fullrobotassem gazebo.launch.py
```

**Mapping with slam_toolbox (in simulation):**

```bash
ros2 launch fullrobotassem mapping.launch.py
```

To drive the robot while mapping, open another terminal and run:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Autonomous navigation with Nav2:**

```bash
ros2 launch fullrobotassem nav2_auto.launch.py
```

**Maze navigation:**

```bash
ros2 launch fullrobotassem maze_navigation.launch.py
```

**Pick and place (box pickup request):**

```bash
ros2 launch fullrobotassem pick_and_place.launch.py pick_x:=2.875 pick_y:=0.530 pick_yaw:=2.795
```

In another terminal, send a pick request either way:

```bash
ros2 topic pub --once /pick_box_request std_msgs/msg/String \
  "{data: '{\"x\": 2.875, \"y\": 0.530, \"yaw\": 2.795, \"qr_code\": \"mmb-6769\"}'}"

python3 ~/ag/src/fullrobotassem/scripts/request_pick.py --qr mmb-6769
```

**Graphics note:** On a laptop with NVIDIA Optimus graphics, Gazebo may need GPU offload. Check the renderer with:

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia glxinfo | grep -i "opengl renderer"
```

Then prefix the launch command with the same two variables. On other machines, skip this.

---

# PART B: Real robot (Raspberry Pi 4 + LD14 lidar)

This part sets up the LD14 lidar and shows its scan in RViz. The motors and wheels need the hardware bridge described below.

### B.1 Connect the sensor

- Power the LD14 from a 5 V supply that can deliver enough current. A powered USB hub or a separate supply works better than the Pi's USB port.
- Connect the sensor's data lines to a CP2102 USB-serial adapter: **sensor TX → adapter RX**, and share the ground between the sensor, adapter, and Pi.
- Plug the adapter into the Pi. Confirm the device name:

```bash
ls -l /dev/ttyUSB*
sudo dmesg | tail -5        # should show "cp210x converter now attached to ttyUSB0"
```

- Give your user access to the serial port (log out and back in afterward):

```bash
sudo usermod -aG dialout $USER
```

- Optional: create a stable name `/dev/ldlidar` with a udev rule. Find the IDs with `udevadm info -a -n /dev/ttyUSB0`, then:

```bash
sudo tee /etc/udev/rules.d/99-ldlidar.rules > /dev/null <<'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="ldlidar", MODE="0666"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### B.2 Test the raw data before ROS

This check confirms the baud rate. The `0x54` byte is the LD14 packet header.

```bash
python3 - <<'EOF'
import serial
for baud in (115200, 230400):
    s = serial.Serial('/dev/ttyUSB0', baud, timeout=1)
    data = s.read(2000)
    s.close()
    print(f"{baud}: {len(data)} bytes, header 0x54 found: {0x54 in data}")
EOF
```

Use the baud rate that reports `header 0x54 found: True`. In testing with this robot's sensor, that was 115200.

### B.3 Install the LD14 ROS 2 driver

The workspace used for testing is `~/ros2_ws`, with the driver package `ldlidar_sl_ros2` in its `src` folder:

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <LDLIDAR_DRIVER_REPOSITORY_URL> ldlidar_sl_ros2
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

Replace `<LDLIDAR_DRIVER_REPOSITORY_URL>` with the URL of the LDROBOT ROS 2 driver you use. The folder name must match the package name in its `package.xml`. Check it with `grep -m1 "<name>" ~/ros2_ws/src/ldlidar_sl_ros2/package.xml`.

### B.4 Set the port, baud rate, and frame

Open `~/ros2_ws/src/ldlidar_sl_ros2/launch/ld14.launch.py` and check these values:

| Parameter | Value |
|---|---|
| `port_name` | `/dev/ttyUSB0` (or `/dev/ldlidar` if you created the udev rule) |
| `serial_baudrate` | `115200` for this sensor (from step B.2) |
| `frame_id` | `lidar_link` (matches the URDF) |
| `laser_scan_topic_name` | `scan` |

The URDF defines `lidar_link` through `lidar_joint`, so the driver should not also publish a static `base_link → base_laser` transform. Remove any static transform for the lidar from the driver's launch file. Then rebuild:

```bash
cd ~/ros2_ws
colcon build --packages-select ldlidar_sl_ros2 --symlink-install
source install/setup.bash
```

### B.5 Run the lidar

Terminal 1, keep it open while you use the lidar:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ldlidar_sl_ros2 ld14.launch.py
```

Look for `ldlidar node start is success` in the output. If you see `serial is not open`, check the port with `ls -l /dev/ttyUSB*`.

Terminal 2, check the data:

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic list | grep scan
timeout 10 ros2 topic hz /scan                                  # about 6 Hz
timeout 10 ros2 topic echo /scan --once --field header.frame_id # lidar_link
```

### B.6 Show the robot and lidar in RViz

Start the lidar first (B.5), then open a new terminal:

```bash
source ~/ros2_ws/install/setup.bash
source ~/ag/install/setup.bash
ros2 launch fullrobotassem display.launch.py
```

In RViz:

1. **Global Options → Fixed Frame:** `base_footprint`.
2. **Add → RobotModel:** Description Source `Topic`, Description Topic `/robot_description`.
3. **Add → LaserScan:** Topic `/scan`. Set Size to `0.03`, Position Transformer to `XYZ`, and Color Transformer to `FlatColor` (set the color to red) or `Intensity`.
4. **Views → Target Frame:** `lidar_link`, with Distance around `1.5` to focus on the sensor.
5. **File → Save Config As:** save as `~/robot_model.rviz`. Reopen it later with:

```bash
rviz2 -d ~/robot_model.rviz
```

Check the TF link if the scan appears in the wrong place:

```bash
timeout 5 ros2 run tf2_ros tf2_echo base_footprint lidar_link
```

Expect a transform with a translation of about `0.14 0.00 0.16` in the setup tested here. The `Invalid frame ID` line that may appear first is a startup message, not an error.

### B.7 Startup order

1. Power the lidar and confirm the port.
2. Start the lidar driver (B.5).
3. Start `display.launch.py` (B.6).
4. Start the hardware bridge (below), when it is available.

---

# PART C: Hardware bridge (Raspberry Pi → Arduino)

Replaces, on real hardware, what `ros_gz_bridge` + `bridge.yaml` did in
Gazebo: it is the only piece that talks to actual motors/servos.

```
ROS2 (Pi)  --UART 115200--> Arduino 1 (master) --I2C 100kHz--> Arduino 2 (slave)
                                   |                                  |
                          PCA9685 (servos: arms)              BTS7960 x2 + JGA37 x2
                          BTS7960 #3 (lift) + 2 limit sw.          (wheels + encoders)
```

### Files
| File | Runs on | Purpose |
|---|---|---|
| `arduino1_master.ino` | Arduino 1 | UART bridge to the Pi, drives arm servos + lift motor, I2C master to Arduino 2 |
| `arduino2_slave.ino` | Arduino 2 | I2C slave, closed-loop wheel speed control + quadrature encoders |
| `hardware_bridge_node.py` | Raspberry Pi (ROS2) | Subscribes to `/cmd_vel`, `/left_arm/cmd_pos`, `/right_arm/cmd_pos`, `/upper_body/cmd_pos`; publishes `/odom`, `/joint_states`, limit-switch/lift status |

### Wiring (from `ketnoi.xlsx`)

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

### Protocols
See the header comments in `arduino1_master.ino` (UART ASCII lines) and
`arduino2_slave.ino` (I2C byte format) for the exact message layout.

### Calibration before trusting odometry or lift speed
- `TICKS_PER_REV` in `arduino2_slave.ino`: spin one wheel exactly one
  revolution by hand, count the ticks, and mirror the value in
  `hardware_bridge_node.py`'s `_update_odometry`.
- `KP` / `KI` in `arduino2_slave.ino`: start low, raise until the wheel
  tracks a step speed command without much overshoot.
- `LIFT_PWM_SPEED` / `LIFT_TIMEOUT_MS` in `arduino1_master.ino`: set the
  timeout longer than the real travel time between limit switches.

### Deploying
1. Flash `arduino1_master.ino` to Arduino 1 and `arduino2_slave.ino` to
   Arduino 2 (Arduino IDE; `arduino1_master.ino` needs the *Adafruit PWM
   Servo Driver Library*).
2. Wire everything per the tables above.
3. On the Pi, add `hardware_bridge_node.py` as a `Node` in your real-robot
   launch file, in place of the `ros_gz_sim`/bridge nodes used by
   `pick_and_place.launch.py` in simulation. `box_picker_node.py` and
   `request_pick.py` need no changes.

## Troubleshooting

| Symptom | Fix |
|---|---|
| `CMakeCache.txt directory ... is different` | Remove `build/ install/ log/` in `~/ag` and rebuild. |
| `Package 'xxx' not found` | Source `install/setup.bash` in this terminal, and check the package name with `ros2 pkg list`. |
| `serial is not open` in the lidar log | Wrong port in `ld14.launch.py`, or no permission. Check `ls -l /dev/ttyUSB*` and the `dialout` group. |
| `/scan` publishes but RViz shows nothing | Set LaserScan Size to `0.03`, Color Transformer, and Target Frame `lidar_link`. |
| RobotModel missing in RViz | Set Description Topic to `/robot_description` and Fixed Frame to `base_footprint`. |
| TF frame missing | Make sure `display.launch.py` is running, and that only one source publishes `base_link → lidar_link`. |
| Gazebo window is black or very slow | Use the GPU offload command in Part A, or lower the simulation rate. |

## Known gaps in this repo

- `hardware_bridge_node.py` is described in Part C but is not in `src/fullrobotassem/scripts/`.
- The Arduino sketches are named `arduino1.ino` / `arduino2.ino` in `program/main/`, while this README refers to `arduino1_master.ino` / `arduino2_slave.ino`.
- `display.launch.py` contains a comment that says `cd ~/trinh/ag`. Use `~/ag` instead.


### Manual keyboard control with teleop_twist_keyboard

The `teleop_twist_keyboard` package provides manual control using a keyboard. It publishes `geometry_msgs/msg/Twist` messages to `/cmd_vel`.

The same keyboard interface can control the robot in both Gazebo simulation and on the real robot.

#### Keyboard layout

Run the teleoperation node in a terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use the following keys to control the robot:

```text
        u    i    o
        j    k    l
        m    ,    .

i : Move forward
, : Move backward
j : Rotate left
l : Rotate right
k : Stop
u / o : Forward with left / right rotation
m / . : Backward with left / right rotation
```

Speed adjustment:

| Key | Function |
|---|---|
| `q` / `z` | Increase / decrease both linear and angular speeds |
| `w` / `x` | Increase / decrease linear speed |
| `e` / `c` | Increase / decrease angular speed |
| `Ctrl+C` | Exit teleoperation |

Keep the teleoperation terminal focused while pressing the movement keys.

#### A. Manual control in Gazebo simulation

1. Start Gazebo:

```bash
source ~/ag/install/setup.bash
ros2 launch fullrobotassem gazebo.launch.py
```

2. In another terminal, start keyboard teleoperation:

```bash
source ~/ag/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

3. Use the keyboard layout above to move the simulated robot.

For mapping or navigation, use the appropriate launch file and make sure only the intended controller is publishing movement commands to `/cmd_vel`.

#### B. Manual control on the real robot (Raspberry Pi)

The real robot uses `robot_hardware/serial_bridge` to convert ROS 2 velocity commands into the existing Arduino UART protocol.

Control flow:

```text
Keyboard
   |
   v
teleop_twist_keyboard
   |
   | geometry_msgs/msg/Twist
   v
/cmd_vel
   |
   v
robot_hardware/serial_bridge
   |
   | UART 115200
   | V <linear_mm_s> <angular_mrad_s>
   v
Arduino 1 (Master)
   |
   | I2C
   v
Arduino 2 (Slave)
   |
   v
Wheel motor drivers
```

**Terminal 1 – Start the hardware bridge**

```bash
cd ~/ag
source /opt/ros/jazzy/setup.bash
source ~/ag/install/setup.bash

ros2 run robot_hardware serial_bridge
```

**Terminal 2 – Start keyboard teleoperation**

```bash
source /opt/ros/jazzy/setup.bash
source ~/ag/install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Terminal 3 – Optional: monitor velocity commands**

```bash
source /opt/ros/jazzy/setup.bash
source ~/ag/install/setup.bash

ros2 topic echo /cmd_vel
```

Press `i` in Terminal 2 to move forward. The hardware bridge converts the ROS 2 velocity into the Arduino command format.


```bash
cd ~/ag
colcon build --packages-select fullrobotassem --symlink-install
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ag/install/setup.bash
ros2 launch fullrobotassem mapping_all.launch.py
```


```bash
cd ~/ag
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ag/install/setup.bash
ros2 launch fullrobotassem mapping_all.launch.py
```

```bash
cd ~/ag

ros2 run nav2_map_server map_saver_cli \
  -f ~/ag/maps/my_map \
  --ros-args -p save_map_timeout:=10.0
```

```bash
cd ~/ros2_ws
colcon build --packages-select ldlidar_sl_ros2
source install/setup.bash
ros2 launch ldlidar_sl_ros2 ld14.launch.py
```


Example:

| ROS 2 velocity | Arduino UART command |
|---|---|
| `linear.x = 0.15`, `angular.z = 0.0` | `V 150 0` |
| `linear.x = 0.0`, `angular.z = 0.3` | `V 0 300` |
| `linear.x = 0.1`, `angular.z = 0.2` | `V 100 200` |
| Both velocities are zero | `V 0 0` |

The bridge currently applies these limits:

| Parameter | Value |
|---|---:|
| Maximum linear velocity | 0.2 m/s |
| Maximum angular velocity | 1.0 rad/s |
| Serial baud rate | 115200 |
| Command transmission frequency | 20 Hz |
| Command timeout | 0.3 seconds |

If no new velocity command arrives within the timeout, the bridge automatically sends `V 0 0` to stop the robot.

**Stopping the robot**

- Press `k` to publish a zero-velocity command.
- Press `Ctrl+C` to exit keyboard teleoperation.
- If the teleoperation node stops publishing, the hardware bridge automatically sends zero velocity after the command timeout.

**Important:** Test manual control with the robot wheels safely lifted or motor power disabled before operating it on the ground. Do not run keyboard teleoperation and autonomous Nav2 control simultaneously unless you have explicitly configured a controller-selection or velocity-multiplexing mechanism.




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