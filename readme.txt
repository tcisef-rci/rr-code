The Rescue Rangers system is composed of several hardware units to run a specific code on each of them. 

Arduino IDE 2.1.0 is used for write and test code for Arduino compatible boards.
OpenMV IDE 4.1.5 is used to write and test code for OpenMV H7 camera.


ATmega328P Arduino compatible boards
rescue_rangers_amr.ino
rescue_rangers_uav.ino
openmv_thermal_camera_face_detection_amr_remote_control.ino //for testing only

ESP8266 compatible boards
esp8266_uav_remote_control.ino

OpenMV H7 board
popular_features_as_the_remote_device_1.py
face_detection.py //for testing only

The search unit (AMR) has UNO R3 + WiFi ATmega328P+ESP8266 (32Mb memory) Arduino compatible board that has two modules. 
The first, ATmega328P based module, needs rescue_rangers_amr.ino uploaded to memory. 
The second, ESP8266 based module, needs esp8266_uav_remote_control.ino code uploaded to memory. 
Programming modules can be done using appropriate deep switch configuration on the board. 
When the search unit equipped with the thermal camera, the openMV board needs to be programmed with MicroPython popular_features_as_the_remote_device_1.py code and downloaded to its root folder as main.py file.

The rescue unit (UAV) Tello drone does not need any configuration. 
However, it can be programmed to run fully autonomous by a tiny ESP32-C3 board programmed to remotely control its locomotions.
In this case ESP32-C3 board needs rescue_rangers_uav.ino code uploaded to its memory. 