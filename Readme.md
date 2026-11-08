ag/
├── build/
├── cad/
├── CardboardBox.v1i.yolov8/
├── curcuit/
├── install/
├── log/
├── model/
│   ├── cardbox_train/
│   ├── runs/
│   ├── venv/
│   ├── best.pt
│   ├── predict_webcam.py
│   ├── qr.png
│   ├── qrgenerator.py
│   ├── results.png
│   ├── train.py
│   └── yolov8n.pt
├── program/
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
│       │   ├── __pycache__/
│       │   ├── display.launch
│       │   ├── display.launch.py
│       │   ├── gazebo.launch
│       │   ├── gazebo.launch.py
│       │   ├── mapping.launch.py
│       │   ├── maze_navigation.launch.py
│       │   ├── nav2_auto.launch.py
│       │   └── pick_and_place.launch.py
│       ├── maps/
│       ├── meshes/
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
│       ├── .gitignore
│       ├── CMakeLists.txt
│       ├── gen_inertial.py
│       ├── package.xml
│       └── wheel_center.py
└── .gitignore



ros2 topic pub --once /pick_box_request std_msgs/msg/String "{data: '{\"x\": 2.875, \"y\": 0.530, \"yaw\": 2.795, \"qr_code\": \"mmb-6769\"}'}"

__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia glxinfo | grep -i "opengl renderer"
ros2 launch fullrobotassem pick_and_place.launch.py pick_x:=2.875 pick_y:=0.530 pick_yaw:=2.795

python3 request_pick.py --qr mmb-6769
