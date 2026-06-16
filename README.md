===================================================================
      AI AUTO CAM  (COMPUTER VISION CAMERAMAN)
                        README DOCUMENT
===================================================================

1. PROJECT OVERVIEW
-------------------
This project is an intelligent, autonomous robotic car chassis designed
to act as a personal cameraman. Powered by an ESP32-CAM, the system 
streams live video wirelessly via Wi-Fi to a host laptop. The laptop 
acts as the "brain," processing the visual data using YOLOv8 and DeepSORT 
to track a specific person dynamically. It calculates real-time tracking 
maneuvers and sends differential drive commands back over low-latency 
UDP packets.

KEY FEATURES:
* Pure Wireless Architecture: No USB, serial cables, or physical TX/RX 
  wire dependencies between the laptop and the car chassis during operation.
* YOLOv8 Object Detection: High-speed real-time human detection customized 
  to focus entirely on human targets (classes=[0]).
* DeepSORT Tracking: Maintains lock-on identity even if the target is 
  briefly blocked or multiple people enter the frame.
* Smart Hysteresis Distance Control: Maintains a stable 1.5 ft distance 
  cushion using custom bounding box pixel-width mathematics to prevent 
  forward/backward stutter.
* Differential PID Steering: Features continuous visual servo panning and 
  smooth chassis drive curves instead of aggressive spot-pivots to protect 
  tracking frame consistency.


2. HARDWARE REQUIREMENTS
------------------------
* Microcontroller (Chassis): ESP32 / NodeMCU 
* Camera Module: ESP32-CAM (OV2640 Sensor, AI-Thinker form factor)
* Programmer Shield: MB Shield Converter (ESP32-CAM-MB USB Shield)
* Chassis: 2WD / 4WD Smart Car Chassis Kit
* Actuators: Dual DC Motors + Pan/Tilt Servo Setup
* Power Source: 18650 Li-ion Batteries (with appropriate step-down buck)
* Connectivity: Micro-USB Wire (for firmware flashing only)


3. SOFTWARE & DEPENDENCY SETUP (LAPTOP)
---------------------------------------
Make sure you have Python 3.8+ installed on your computer. Install the 
core required computer vision libraries using your terminal/command prompt:

Command:
pip install ultralytics opencv-python numpy deep-sort-realtime


4. ESP32-CAM FIRMWARE & UPLOADING PROCEDURE
-------------------------------------------
The camera runs a specialized wireless server using Espressif's optimized 
production examples. 

Step 1: Open Arduino IDE.
Step 2: Navigate to: File -> Examples -> ESP32 -> Camera -> CameraWebServer
Step 3: In the main configuration tab, uncomment the correct camera profile:
        #define CAMERA_MODEL_AI_THINKER
Step 4: Enter your local Wi-Fi credentials (SSID and Password) in the code.
Step 5: Plug the ESP32-CAM firmly onto the MB Shield Converter.
Step 6: Connect the MB Shield to your laptop using a standard Micro-USB wire.
Step 7: Select "AI Thinker ESP32-CAM" as your board and choose the active COM Port.
Step 8: Upload the sketch.
Step 9: Once successful, open the Serial Monitor using the shortcut:
        - Windows: Ctrl + Shift + M
        - Mac: Cmd + Shift + M
Step 10: Set the Baud Rate to 115200 in the bottom-right corner.
Step 11: Tap the physical "RESET" button on the ESP32-CAM module.
Step 12: Copy the local IP address displayed on the screen (e.g., http://10.205.197.165/).


5. CRITICAL NETWORKING NOTE
---------------------------
* UDP & LOCAL SUBNET ROUTING DYNAMICS: This project utilizes high-speed UDP 
  packets to send rapid movement commands from the laptop to the car.
* Local IP subnets change depending on your Internet Service Provider (ISP), 
  router firewall configurations, or local mobile hotspot parameters.
* Every time you switch to a different network, your router will assign a 
  fresh IP address to both your laptop and the ESP32-CAM. You must always check 
  your Serial Monitor logs and update the host IPs inside your Python script 
  before running the project. If the IP is wrong, the system will fail to connect.


6. HOW TO RUN & OPERATE THE SYSTEM
----------------------------------
Step 1: Power the Vehicle
        Turn on your car chassis battery switch. Ensure the ESP32 and motor 
        controllers boot up and connect to your local network/hotspot.

Step 2: Configure and Launch Python Tracker
        Update the ESP32_CAM_URL and ESP32_UDP_IP variables inside your Python 
        script to match the current network session. Launch the application:
        python main_tracker.py

Step 3: Calibrate the 1.5 ft Distance Bounds
        1. Stand exactly 1.5 feet away from the vehicle's camera array and face it.
        2. Select the active OpenCV display window on your laptop screen.
        3. Press 'c' on your keyboard. 
        4. The system automatically samples your current bounding box dimensions, 
           saves it as the target reference (TARGET_BBOX_W), and locks the deadzones.

KEYBOARD CONTROLS (OPENCV WINDOW):
* Press 'c' -> Calibrate target distance to 1.5 ft.
* Press 'r' -> Reset target locks and PID controllers.
* Press 'q' -> Safely stop vehicle motors and exit the application.


7. REPOSITORY STRUCTURE
-----------------------
├── CameraVision.txt    # Detailed network, hardware flashing, and IP guide
├── main_tracker.py     # Main Python script (YOLOv8 + DeepSORT + UDP Controller)
├── README.txt          # This documentation file
└── firmware/
    └── chassis_udp.ino # Arduino firmware for the car motor receiver

===================================================================
