// Title:  Rescue Rangers Test Code - UAV autonomous locomotion with obstacle avoidance
// Author: Richard Ilioukevitch
// Date:   1 March 2024
// Some part of the code relies on (ESPTelloCLI) and Protractor(Protractor) Arduino libraries
//
// The following code runs on ESP32-C3 Arduino compatible board
// to test obstacle avoidance algorithm
// The board is connected to the Protractor IR proximity sensor
// ESP32-C3 board is attached and powered by the drone.
// Protractor sensor is attached and powered by ESP32-C3 board
// Communication between ESP32-C3 and drone are done over Tello's Wi-Fi network
// Communication between ESP32-C3 and Protractor sensor are over (D4/D5)
// There are four modes of operation:
// Mode 0: setup up mode (connecting to Tello's Wi-Fi network, initialize connecting to the Protractor sensor)
// Mode 1: normal mode (flying by selecting clear path and avoiding obstacles
// Mode 2: reconnecting mode (hoovering while trying to re-establising Wi-Fi connection with the drone)
// Mode 3: landing mode (landing after timeout)

#include <ESPTelloCLI.h>
#include <Protractor.h>
#include <Wire.h>
Protractor protractor;
ESPTelloCLI TelloCLI;

bool Connected;
unsigned int bestPath;
int mode = 0;
int lspeed;
int rspeed; 
int rotation;

void WiFiEvent(WiFiEvent_t event){
    switch(event) {
#ifdef ESP8266
      case WIFI_EVENT_STAMODE_GOT_IP:
#else
      case ARDUINO_EVENT_WIFI_STA_GOT_IP:
#endif
          TelloCLI.begin();
          // Turn off telemetry if not needed.
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

void setup()
{
  char SSID[65];
  //Enable Protractor sensor
  Wire.begin();
  protractor.begin(Wire,69); // Use I2C/Wire Library to talk with Protractor on default address 69
  delay(500);
  // Check communication with the Protractor
  bool connectedProctractor = protractor.read();
  //Connect to Tello network
  Connected = false;
  WiFi.mode(WIFI_STA);
  WiFi.onEvent(WiFiEvent);
  strcpy(SSID, "TELLO-EFD2F2"); //The Network name is specific for each Tello drone model
  WiFi.begin(SSID);
  delay(5000);
  String command;
  //take off the drone after connection is established and switch to mode 2; altitude is 20 cm above ground zero
  if (Connected) {
    command = "command";TelloCLI.write(command.c_str(), command.length()); delay(200);
    command = "rc 0 0 0 0"; TelloCLI.write(command.c_str(), command.length()); delay(100);
    command = "rc -100 -100 -100 100"; TelloCLI.write(command.c_str(), command.length()); delay(1000);
    command = "rc 0 0 0 0"; TelloCLI.write(command.c_str(), command.length()); delay(100);
    command = "rc 0 0 20 0"; TelloCLI.write(command.c_str(), command.length()); delay(3000); 
    command = "rc 0 0 0 0"; TelloCLI.write(command.c_str(), command.length()); delay(100);
    mode = 1;
  }
}

void loop()
{
  String command; 
  char ID[65];
  //Calculate the speed of rotation in horizontal plane by calculating the angle of the best (more clear) path
  //When there is no obstacle within the sensing zone (distance 40 cm, FOV 30 degrees, HOV- 120 degrees) drone will fly straight, speed 10 cm/s
  if (Connected & mode == 1 & millis() < 120000) {
      protractor.read();
      if (protractor.pathCount() > 0) { bestPath = protractor.pathAngle(0); } else { bestPath = 90; } 
      lspeed = 100; rspeed = 100; 
      if (bestPath < 90)       {lspeed = 100 * bestPath         / 90;}         
      else if (bestPath > 90)  {rspeed = 100 * (180 - bestPath) / 90;}
      rotation = lspeed - rspeed;
      if (rotation != 0) command = "rc 0 0 0 " + String(rotation);
      else command = "rc 0 10 0 0";
      TelloCLI.write(command.c_str(), command.length()); delay(50);
      command = "rc 0 0 0 0"; 
      TelloCLI.write(command.c_str(), command.length()); delay(50);
  }
  else {
  // Re-establish connect with Tello network when it is lost
    mode = 2;
    
    if (Connected & millis() > 120000 & mode < 3) {//50000
         //land drone after timeout (mode 3)
         command = "land"; TelloCLI.write(command.c_str(), command.length()); delay(10000); 
         mode = 3;
    }
    // Switch back to mode 1 when re-connected with Tello Network
    if (!Connected & mode < 3) {
         Connected = false;
         WiFi.begin(strcpy(ID, "TELLO-EFD2F2"));
         delay(5000);
         if (Connected & millis() < 110000) {
            command = "command";TelloCLI.write(command.c_str(), command.length()); delay(200);
            mode = 1;
         }
    }
  }
}