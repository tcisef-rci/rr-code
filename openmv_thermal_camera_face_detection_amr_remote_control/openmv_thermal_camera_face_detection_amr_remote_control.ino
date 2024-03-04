// Title:  Rescue Rangers Test Code - AMR autonomous locomotion with obstacle avoidance and the survivor thermal vision face detection (for testing only)
// Author: Richard Ilioukevitch
// Date:   1 March 2024
// Dependances: Some part of the code relies on ZumoShield(Zumo*), Protractor sensor (Protractor) Arduino libraries and openMV camera RPC and FLIR Lepton libraries
// and has been adopted from their example codes.
//
// The following code runs on Arduino compatible board (based on the ATmega328P) 
// to test the search and obstacle avoidance behaviour for AMR unit 
// supplemented with AI thermal vision face detection MicroPython code run on OpenMV cam board
// Arduino board is attached and powered by ZumoShield board 
// Protractor sensor is attached and powered by ZumoShield board
// Communication between Arduino and Protractor sensor is over SDA/SCL
// Communication between Arduino and OpenMV thermal camera is over TX/RX
// There are three modes of operation:
// Mode 0: the survivor search based on detecting his/her IR beaker emission on ground
// Mode 1: the drone search based on detecting drone its IR beaker emission in mid-air
// Mode 2: the drone precision landing to the survivor location
// the code will only test AMR behavior in Mode 0 as a possible alternative for detecting the survivor IR beaker emission by the PixyCam

#include <ZumoMotors.h>
#include <ZumoShield.h>
#include <avr/pgmspace.h>
#include <Wire.h>
#include <Protractor.h>
#include <SPI.h> 
 
//used for remote control of openMV camera to run face detection procedure
#include <openmvrpc.h>
openmv::rpc_scratch_buffer<256> scratch_buffer; // All RPC objects share this buffer.
openmv::rpc_hardware_serial_uart_master interface(115200);

#define SPEED           200 
#define FULL_SPEED      400
#define LANDDRONE       2
#define FINDDRONE       1
#define FINDVICTIM      0

ZumoMotors motors; 
ZumoBuzzer buzzer; 
Protractor protractor; 

int mode;

//Initiate communication with Protractor proximity sensor
//Initiate the survivor search mode
//Initiate serial communication with openMV camera board

void setup() {
  Wire.begin(); 
  interface.begin();
  Serial.begin(115200);
  protractor.begin(Wire,69);
  int protractorConnected = protractor.read(0); 
  mode = FINDVICTIM;
}

//procedure to invoke remotely face detection on openMV camera board
bool exe_face_detection()
{  
    struct { uint16_t x, y, w, h; } face_detection_result;
    return interface.call_no_args(F("face_detection"), &face_detection_result, sizeof(face_detection_result));
}

void loop()
{  int speed;

   if (mode == FINDVICTIM){ 
	protractor.read();
	int leftMotorSpeed;
	int rightMotorSpeed; 
	unsigned int bestPath;
	exe_face_detection(); // Face should be about 2ft away.
		
	if (protractor.pathCount() > 0) { 
		bestPath = protractor.pathAngle(0); 
	} 
	else { 
		bestPath = 90; 
	} 
	leftMotorSpeed = FULL_SPEED; 
	rightMotorSpeed = FULL_SPEED; 

	if (bestPath < 90)  {
		leftMotorSpeed = FULL_SPEED * bestPath / 90;
	} 
	else if (bestPath > 90)  {
		rightMotorSpeed = FULL_SPEED * (180 - bestPath) / 90;
	}

        // when the survivor face is detected stop the AMR motors
	if (exe_face_detection()) { 
		leftMotorSpeed = 0; rightMotorSpeed = 0;
	}
    motors.setSpeeds(leftMotorSpeed, rightMotorSpeed);
  }
}