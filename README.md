# This is a ROS 2 project for an autonomous robot that can track and follow a person wearing a specific colored shirt using a YOLOv5-powered vision system. It features multiple control modes, including manual joystick control and autonomous following, integrated with a safety laser scanner.

## 🎥 Demo
<video controls src="pal.mp4" title="PAL Robot Demo"></video>

## 📁 Workspace Structure

    pal_ws/
    ├── src/
    │   ├── robot/                # Main robot controller (main.cpp, launch, etc)
    │   ├── device/               # Hardware interface nodes (gpio, laser, motor, joyAndro)
    │   ├── camera/               # YOLOv5 vision node (yolov5.cpp, launch, config)
    │   ├── robot_interfaces/     # Custom ROS2 messages (JoyAndro.msg)
    ├── start.sh                  # Bash script for setup IO and robotPAL.service 
    └── README.md

## 📦 Custom Message

**robot_interfaces/msg/JoyAndro.msg**
```text
std_msgs/Header header  
float32 x  
float32 y  
bool button_x  
bool button_circle  
bool button_triangle  
bool button_square  
```


## 🧠 Node Overview
A.  robot/main_node (main.cpp)
    -   Fungsi: Node utama pengendali robot.
    -   Mode:
        -   IDLE: Robot berhenti.
        -   MANUAL: Kontrol penuh via joystick Android (JoyAndro).
        -   AUTO: Person following (mengikuti orang dengan YOLO).
    -   Kontrol:
        -   Joystick X/Y: Linear/angular velocity.
        -   Button X: Masuk mode AUTO.
        -   Button O: Masuk mode MANUAL.
        -   Button Triangle: Masuk mode IDLE.
        -   Button Square: Boost kecepatan saat ditekan.
    -   Topic yang digunakan:
        -   /device/joyAndro (sub): Data joystick Android.
        -   /camera/detections (sub): Data deteksi YOLO.
        -   /device/laser/distances (sub): Data sensor jarak.
        -   /device/motor/encoder, /rpm, /vbus (sub): Data motor.
        -   /device/motor/cmd_vel (pub): Perintah kecepatan motor.

B.  device/joyandro_node (joyAndro.cpp)
    -   Fungsi: Node pembaca joystick Android via Bluetooth.
    -   Publish:
        -   /device/joyAndro (robot_interfaces/msg/JoyAndro, 100Hz):
            -   x, y: posisi stick
            -   button_x, button_circle, button_triangle, button_square: status tombol
    -   Catatan:
        -   Jika tidak ada input, tetap publish data default (semua 0/false).

C.  device/gpio_node
    -   Fungsi: (Opsional, bisa di-nonaktifkan) Membaca tombol GPIO hardware.
    -   Publish:
        -   /device/gpio (std_msgs/UInt8MultiArray): Status tombol GPIO.

D.  device/laser_node
    -   Fungsi: Membaca data sensor jarak laser (serial/modbus).
    -   Publish:
        -   /device/laser/distances (std_msgs/Float32MultiArray): Data 5 probe sensor.

E.  device/zlac8015_node
    -   Fungsi: Driver motor CAN.
    -   Subscribe:
        -   /device/motor/cmd_vel (geometry_msgs/Twist): Perintah kecepatan.
    -   Publish:
        -   /device/motor/encoder, /rpm, /vbus: Data status motor.

F.  camera/yolo_node (yolov5.cpp)
    -   Fungsi: Deteksi orang berbaju biru dengan YOLOv5 + filter HSV.
    -   Publish:
        -   /camera/detections (std_msgs/String): Data deteksi orang (format: "person_blue:confidence:x:y:width:height").
        -   /camera/image_annotated (sensor_msgs/Image): Gambar dengan bounding box.
    -   Subscribe:
        -   /camera/image_raw (opsional): Untuk mode input dari topic.

## Topic List & Penjelasan
Topic	                Type	                        Direction	Keterangan
/device/joyAndro	    robot_interfaces/msg/JoyAndro	pub	        Joystick Android
/device/gpio	        std_msgs/UInt8MultiArray	    pub	        Tombol GPIO 
/device/laser/distances	std_msgs/Float32MultiArray	    pub	        Sensor jarak
/device/motor/cmd_vel	geometry_msgs/Twist	            sub	        Kecepatan motor
/device/motor/encoder	std_msgs/Int32MultiArray	    pub	        Data encoder motor
/device/motor/rpm	    std_msgs/Float32MultiArray	    pub	        Data RPM motor
/device/motor/vbus	    std_msgs/Float32	            pub	        Tegangan bus motor
/camera/detections	    std_msgs/String	                pub	        Data deteksi YOLO
/camera/image_annotated	sensor_msgs/Image	            pub	        Gambar hasil deteksi


## Cara Menjalankan Program
A.  Build Workspace
    cd ~/pal_ws
    colcon build --symlink-install
    atau
    colcon build --symlink-install --packages-select nama_package
    source install/setup.bash

B.  Jalankan Device 
    ros2 launch device device.launch.py
    -   Akan menjalankan: gpio_node (opsional), joyandro_node, zlac8015_node, laser_node.

C.  Jalankan Kamera (YOLO)
    ros2 launch camera camera.launch.py
    Akan menjalankan: yolo_node (YOLOv5 deteksi orang berbaju biru).

D.  Jalankan Main Robot Controller
    ros2 launch robot robot.launch.py

E.  Monitoring
    Joystick:
        ros2 topic echo /device/joyAndro
    Motor Command:
        ros2 topic echo /device/motor/cmd_vel
    YOLO Detection:
        ros2 topic echo /camera/detections
    Laser:
        ros2 topic echo /device/laser/distances

F.  Web Video Stream
    Jika menggunakan web_video_server:
    http://<robot_ip>:8080/stream?topic=/camera/image_annotated

## Kontrol
Manual (Joystick Android):
    Stick X/Y: Linear/angular velocity
    X: AUTO mode (person following)
    O: MANUAL mode (joystick)
    Triangle: IDLE mode (stop)
    Square: BOOST (kecepatan dobel saat ditekan)
AUTO: Robot mengikuti orang berbaju biru (YOLO + laser safety)

## Konfigurasi Warna (color_config.yaml)
    File: color_config.yaml
        Mengatur HSV threshold untuk deteksi biru (default) pada YOLO.
