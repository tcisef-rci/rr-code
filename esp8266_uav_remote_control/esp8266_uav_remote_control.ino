// Title:  Rescue Rangers Test Code - AMR remote control of the UAV over Tello Wi-Fi network
// Author: Richard Ilioukevitch
// Date:   1 March 2024
// Dependances: Some part of the code relies on ESPTelloCli Arduino libraries
// and has been adopted from their example codes.
//
// The following code runs on Arduino ESP8266 compatible board 
// to test its ESP8266 module remote control of the Tello drone over Wi-Fi network
// Communication between Arduino and ESP8266 is over hardware serial port
// Communication between ESP8266 module and Tello drone over Wi-Fi
// The code needs to be uploaded to ESP8266 module prior to running main program (rescue_rangers_amr.ino) on Arduino ESP8266 compatible board

#include <ESPTelloCLI.h>
ESPTelloCLI TelloCLI;

bool Connected;
String command;
char SSID[65];
String incomingString;

void WiFiEvent(WiFiEvent_t event){
    switch(event) {
#ifdef ESP8266
      case WIFI_EVENT_STAMODE_GOT_IP:
#else
      case ARDUINO_EVENT_WIFI_STA_GOT_IP:
#endif
          TelloCLI.begin();
          TelloCLI.setTelemetry(false);
          Connected = true;
          break;
#ifdef ESP8266
      case WIFI_EVENT_STAMODE_DISCONNECTED:
#else
      case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
#endif
          TelloCLI.end();
          Connected = false;
          break;
      default: break;
    }
}

void setup() {
 char SSID[65];
  //set up serial communication speed between Arduino and ESP8266 module
  Serial.begin(115200);
  Serial.setTimeout(0);
  while (! Serial);
  Connected = false;
  //set up connection between ESP8266 module and Tello drone
  WiFi.mode(WIFI_STA);
  WiFi.onEvent(WiFiEvent);
  strcpy(SSID, "TELLO-5D272D"); //The Network name is specific for each Tello drone model
  WiFi.begin(SSID);
  delay(5000);
  String command;
  if (Connected) {
     //activate remote control for Tello drone 
     command = "command"; TelloCLI.write(command.c_str(), command.length());delay(100);
  }
}

void loop() {
  //Listen for AMR unit serial messages for the drone remote control 
  if (Serial.available() > 0){
    command = Serial.readString();
    TelloCLI.write(command.c_str(), command.length());delay(100);
  }
}