# robotic-arm

Executing this project involves setting up the hardware, programming the microcontroller, and then running the computer-side script to send commands.

Here is a detailed, step-by-step guide to run the entire system.

Phase 1: Hardware Assembly and Wiring
This is the most critical phase. Incorrect wiring can damage your components.

Required Components:

Your 6-axis robotic arm with 6 servo motors.

An ESP32-CAM module.

An external power supply for the servos. This is not optional. The ESP32 cannot provide enough current for six motors. You need something like a 5V/6V, 3A+ power adapter or a Battery Eliminator Circuit (BEC).

An FTDI (Future Technology Devices International) programmer to upload code to the ESP32-CAM.

Jumper wires.

Wiring Instructions:

Power the Servos:

Connect the VCC (red) wire of ALL six servos to the positive (+) terminal of your external power supply.

Connect the GND (black/brown) wire of ALL six servos to the negative (-) terminal of your external power supply.

Connect Servos to ESP32:

Connect the Signal (orange/yellow) wire of each servo to the corresponding GPIO pin on the ESP32 that you defined in the code.

Base Servo Signal -> GPIO 12

Shoulder Servo Signal -> GPIO 13

Elbow Servo Signal -> GPIO 14

Wrist Pitch Servo Signal -> GPIO 15

Wrist Roll Servo Signal -> GPIO 27

Gripper Servo Signal -> GPIO 26

Create a Common Ground:

Connect the GND pin on the ESP32 to the negative (-) terminal of your external power supply.

Why this is crucial: All components in a circuit must share a common ground reference for signals to be interpreted correctly. Without this, the servos will not respond properly.

Prepare for Uploading:

Connect the FTDI programmer to your ESP32-CAM. The typical connections are:

FTDI 5V -> ESP32 5V

FTDI GND -> ESP32 GND

FTDI TX -> ESP32 U0R (RX)

FTDI RX -> ESP32 U0T (TX)

Phase 2: Setting Up the Software Environment
Install Arduino IDE: If you don't have it, download and install the latest version from the official Arduino website.

Add ESP32 Board Manager:

Open Arduino IDE. Go to File > Preferences.

In the "Additional Boards Manager URLs" field, paste the following URL:

https://dl.espressif.com/dl/package_esp32_index.json
Click "OK".

Install ESP32 Boards:

Go to Tools > Board > Boards Manager....

Search for "esp32" and install the package by Espressif Systems.

Install Libraries:

Go to Tools > Manage Libraries....

Search for and install the following library:

ESP32Servo by Kevin Harrington

Phase 3: Programming the ESP32
Configure the Code:

Open the C++ code provided earlier in your Arduino IDE.

This is the most important step. Carefully update the configuration section at the top of the file:

ssid and password: Enter your home or work Wi-Fi network name and password.

L1, L2, L3: You must measure your physical robotic arm's link lengths (in millimeters) and update these constants. The default values are just placeholders. The robot will not move correctly if these are wrong.

SERVO_PIN_...: Double-check that these pin numbers match your wiring from Phase 1.

CAMERA_MODEL_...: Ensure the line for your specific camera model (e.g., CAMERA_MODEL_AI_THINKER) is uncommented and others are commented out.

Upload the Code:

Connect the FTDI programmer (which is connected to your ESP32-CAM) to your computer's USB port.

In the Arduino IDE, go to Tools > Board and select "AI-Thinker ESP32-CAM".

Go to Tools > Port and select the COM port that your FTDI programmer is connected to.

Enter Flashing Mode: Connect a jumper wire from pin GPIO 0 to GND on the ESP32-CAM.

Press the small reset button on the back of the ESP32-CAM.

Click the Upload button (the right-arrow icon) in the Arduino IDE.

Once you see the "Connecting..." message in the console, the upload will begin.

After the upload is complete, disconnect the jumper wire from GPIO 0 and GND, then press the reset button again. This puts the board in normal run mode.

Get the ESP32's IP Address:

Keep the ESP32 connected to your computer.

In the Arduino IDE, go to Tools > Serial Monitor.

Set the baud rate to 115200.

You should see the boot-up messages, the Wi-Fi connection progress, and finally, the IP address. It will look something like this:

WiFi connected.
IP Address: 192.168.1.100
HTTP server started.
System Ready.
Note down this IP address.

Phase 4: Running the Python Controller
Install Python: If you don't have it, install Python from the official Python website.

Inst