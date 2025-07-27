# 🤖 Person-Following Robot with Sound System

A ROS 2 project for an autonomous robot that can track and follow a person wearing a specific colored shirt using a YOLOv5-powered vision system. It features multiple control modes, including manual joystick control and autonomous following, integrated with a safety laser scanner.

---

## 🎥 Demo
![Robot Demo](pal.gif)

---

## 📁 Workspace Structure

```bash
pal_ws/
├── src/
│   ├── robot/                # Main robot controller (main.cpp, launch, etc)
│   ├── device/               # Hardware interface nodes (gpio, laser, motor, joyAndro)
│   ├── camera/               # YOLOv5 vision node (yolov5.cpp, launch, config)
│   └── robot_interfaces/     # Custom ROS2 messages (JoyAndro.msg)
├── start.sh                  # Bash script for setup IO and robotPAL.service
└── README.md
```

---

## 📦 Custom Message

**`robot_interfaces/msg/JoyAndro.msg`**
```text
std_msgs/Header header  
float32 x  
float32 y  
bool button_x  
bool button_circle  
bool button_triangle  
bool button_square
```

---

## 🧠 Node Overview

* **A. `robot/main_node` (main.cpp)**
    * **Function**: The main robot controller node.
    * **Modes**:
        * `IDLE`: The robot is stationary.
        * `MANUAL`: Full control via an Android joystick (JoyAndro).
        * `AUTO`: Person following using YOLO.
    * **Controls**:
        * Joystick X/Y: Controls linear/angular velocity.
        * Button X: Enters `AUTO` mode.
        * Button O: Enters `MANUAL` mode.
        * Button Triangle: Enters `IDLE` mode.
        * Button Square: Engages a speed boost when pressed.
    * **Topics Used**:
        * `/device/joyAndro` (sub): Data from the Android joystick.
        * `/camera/detections` (sub): Detection data from YOLO.
        * `/device/laser/distances` (sub): Data from the distance sensor.
        * `/device/motor/encoder`, `/rpm`, `/vbus` (sub): Feedback from the motors.
        * `/device/motor/cmd_vel` (pub): Velocity commands for the motors.

* **B. `device/joyandro_node` (joyAndro.cpp)**
    * **Function**: Reads input from an Android joystick via Bluetooth.
    * **Publishes**:
        * `/device/joyAndro` (`robot_interfaces/msg/JoyAndro`, 100Hz):
            * `x`, `y`: Stick position.
            * `button_x`, `button_circle`, `button_triangle`, `button_square`: Button states.
    * **Note**: If there is no input, it continues to publish default data (all values 0 or false).

* **C. `device/gpio_node`**
    * **Function**: (Optional, can be disabled) Reads the state of hardware GPIO buttons.
    * **Publishes**:
        * `/device/gpio` (`std_msgs/UInt8MultiArray`): GPIO button status.

* **D. `device/laser_node`**
    * **Function**: Reads data from the laser distance sensor (serial/modbus).
    * **Publishes**:
        * `/device/laser/distances` (`std_msgs/Float32MultiArray`): Data from the 5 sensor probes.

* **E. `device/zlac8015_node`**
    * **Function**: CAN motor driver.
    * **Subscribes**:
        * `/device/motor/cmd_vel` (`geometry_msgs/Twist`): Velocity commands.
    * **Publishes**:
        * `/device/motor/encoder`, `/rpm`, `/vbus`: Motor status data.

* **F. `camera/yolo_node` (yolov5.cpp)**
    * **Function**: Detects a person in a blue shirt using YOLOv5 + HSV filter.
    * **Publishes**:
        * `/camera/detections` (`std_msgs/String`): Detection data (format: `"person_blue:confidence:x:y:width:height"`).
        * `/camera/image_annotated` (`sensor_msgs/Image`): Image with bounding boxes.
    * **Subscribes**:
        * `/camera/image_raw` (optional): For input mode from a topic.

---

## 📊 Topic List & Descriptions

| Topic                   | Message Type                     | Direction | Description            |
| ----------------------- | -------------------------------- | --------- | ---------------------- |
| `/device/joyAndro`      | `robot_interfaces/msg/JoyAndro`  | `pub`     | Android Joystick Data  |
| `/device/gpio`          | `std_msgs/UInt8MultiArray`       | `pub`     | GPIO Button Status     |
| `/device/laser/distances`| `std_msgs/Float32MultiArray`    | `pub`     | Distance Sensor Data   |
| `/device/motor/cmd_vel` | `geometry_msgs/Twist`            | `sub`     | Motor Velocity Command |
| `/device/motor/encoder` | `std_msgs/Int32MultiArray`       | `pub`     | Motor Encoder Data     |
| `/device/motor/rpm`     | `std_msgs/Float32MultiArray`     | `pub`     | Motor RPM Data         |
| `/device/motor/vbus`    | `std_msgs/Float32`               | `pub`     | Motor Bus Voltage      |
| `/camera/detections`    | `std_msgs/String`                | `pub`     | YOLO Detection Data    |
| `/camera/image_annotated`| `sensor_msgs/Image`             | `pub`     | Annotated Image Feed   |

---

## 🚀 How to Run the Program

* **A. Build the Workspace**
    ```bash
    cd ~/pal_ws
    colcon build --symlink-install
    
    # or build a specific package
    # colcon build --symlink-install --packages-select <package_name>
    
    source install/setup.bash
    ```

* **B. Launch Devices**
    ```bash
    ros2 launch device device.launch.py
    ```
    *This will run: `gpio_node` (optional), `joyandro_node`, `zlac8015_node`, `laser_node`.*

* **C. Launch Camera (YOLO)**
    ```bash
    ros2 launch camera camera.launch.py
    ```
    *This will run: `yolo_node` (YOLOv5 detection for people in blue shirts).*

* **D. Launch the Main Robot Controller**
    ```bash
    ros2 launch robot robot.launch.py
    ```

* **E. Monitoring**
    * **Joystick**:
        ```bash
        ros2 topic echo /device/joyAndro
        ```
    * **Motor Command**:
        ```bash
        ros2 topic echo /device/motor/cmd_vel
        ```
    * **YOLO Detection**:
        ```bash
        ros2 topic echo /camera/detections
        ```
    * **Laser**:
        ```bash
        ros2 topic echo /device/laser/distances
        ```

* **F. Web Video Stream**
    * If using `web_video_server`:
        `http://<robot_ip>:8080/stream?topic=/camera/image_annotated`

---

## 🎮 Controls

* **Manual (Android Joystick)**:
    * Stick X/Y: Linear/angular velocity
    * X: `AUTO` mode (person following)
    * O: `MANUAL` mode (joystick)
    * Triangle: `IDLE` mode (stop)
    * Square: `BOOST` (doubles speed while pressed)

* **AUTO**:
    * The robot follows a person in a blue shirt (YOLO + laser safety).

---

## ⚙️ Color Configuration (`color_config.yaml`)

* **File**: `color_config.yaml`
* **Function**: Sets the HSV threshold for blue color detection (default) in YOLO.