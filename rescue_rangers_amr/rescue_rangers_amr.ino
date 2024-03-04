// Title:  Rescue Rangers Test Code - AMR autonomous locomotion with obstacle avoidance and the survivor search behaviour
// Author: Richard Ilioukevitch
// Date:   1 March 2024
// Dependances: Some part of the code relies on ZumoShield(Zumo*), PixyCam (Pixy) and Protractor sensor (Protractor) Arduino libraries
// and has been adopted from their example codes.
//
// The following code runs on Arduino compatible board (based on the ATmega328P) 
// to test the search and obstacle avoidance behaviour for AMR unit
// Arduino board is attached and powered by ZumoShield board
// Protractor sensor is attached and powered by ZumoShield board
// Communication between Arduino and PixyCam is over ISP
// Communication between Arduino and Protractor sensor is over SDA/SCL
// There are three modes of operation:
// Mode 0: the survivor search based on detecting his/her IR beaker emission on ground
// Mode 1: the drone search based on detecting drone its IR beaker emission in mid-air
// Mode 2: the drone precision landing to the survivor location

//#include <ZumoBuzzer.h>  
#include <ZumoMotors.h>
#include <ZumoShield.h>
//#include <Pushbutton.h>
#include <avr/pgmspace.h>
#include <Wire.h>
#include <Protractor.h>
#include <SPI.h>  
#include <Pixy.h>
#include <L3G.h>

#define LED_PIN         5    				//pin to control the LED search light
#define SEARCH          700  				//position of the PixyCam tilt servo in the survivor search mode (90 degrees)
#define RESCUE          800  				//position of the PixyCam tilt servo in the drone search mode    (120 degrees)
#define SPEED           200  				//maximum speed of the AMR for gyroscope calibration
#define FULL_SPEED      400  				//maximum speed of the AMR for the driving
#define LANDDRONE       2    				//mode 2 - drone landing
#define FINDDRONE       1    				//mode 1 - search the drone
#define FINDVICTIM      0    				//mode 0 - search the survivor
#define X_CENTER        160L 				//Pixy sensor horizontal resolution
#define Y_CENTER        100L 				//PixyCam sensor vertical resolution 
#define RCS_MIN_POS     700L 				//this PixyCam values used for locking drone position in mid air and 
#define RCS_MAX_POS     1000L 				//to perform guidance for remote controlled precision landing of the drone
#define RCS_CENTER_POS	((RCS_MAX_POS-RCS_MIN_POS)/2)	//coordinates of the PixyCam center

L3G gyro;
ZumoMotors motors; 
ZumoBuzzer buzzer; 
Pushbutton button(ZUMO_BUTTON); 
Protractor protractor; 
Pixy pixy; 

const char sound_effect[] PROGMEM = "O4 T100 V15 L4 MS g12>c12>e12>G6>E12 ML>G2"; 
int mode;
int sampleNum=500; //number of sample readings for calibration of gyroscope
int dc_offset=0;   // used for calibration
double noise=0;    // used for calibration
unsigned long time;
int sampleTime=10;
int rate;
int prev_rate=0;
double angle=0;

//Pan and Tilt servos loop control class for PID positioning of PixyCam
//Adopted from PixyCam library for Arduino
class ServoLoop {
	public: ServoLoop(int32_t pgain, int32_t dgain);
	void update(int32_t error); 
	int32_t m_pos;
	int32_t m_prevError;
	int32_t m_pgain;
	int32_t m_dgain;
};

ServoLoop tiltLoop(-50, -700);                     //set up PID values for tilt servo control of PixyCam

ServoLoop::ServoLoop(int32_t pgain, int32_t dgain) {
	m_pos = RCS_CENTER_POS;
	m_pgain = pgain;
	m_dgain = dgain;
	m_prevError = 0x80000000L;
} 

void ServoLoop::update(int32_t error) {
	long int vel;char buf[32]; 
	if (m_prevError!=0x80000000) {	
		vel = (error*m_pgain + (error - m_prevError)*m_dgain)>>10;m_pos += vel; 
		if (m_pos>RCS_MAX_POS) m_pos = RCS_MAX_POS; 
		else if (m_pos<RCS_MIN_POS) m_pos = RCS_MIN_POS; 
	} m_prevError = error;
}

//Initiate communication with ZumoShield gyroscope, PixyCam, and Protractor proximity sensor
//Calibrate gyroscope by reading sensing magnetic field while spinning AMR
//Initiate the survivor search mode
//Setup PixyCam at 90 degrees search position,
void setup() {

        //Initiate communication with PixyCam
	Wire.begin(); 
	pixy.init(); 
        //Initiate communication with gyro sensor
	Serial.begin(9600); 
	if (!gyro.init()) {
		Serial.println("Failed to autodetect gyro type!");
		while (1); 
	} 
        // Calibrate the gyroscope
	gyro.enableDefault();
	for(int n=0;n<sampleNum;n++){
		gyro.read();
		dc_offset+=(int)gyro.g.z;
	} 
	dc_offset=dc_offset/sampleNum; 
	for(int n=0;n<sampleNum;n++){
		gyro.read(); 
		if((int)gyro.g.z-dc_offset>noise) noise=(int)gyro.g.z-dc_offset;
		else if((int)gyro.g.z-dc_offset<-noise) noise=-(int)gyro.g.z-dc_offset; 
	} 
	noise=noise/100; //gyro returns hundredths of degrees/sec

        //Initiate communication with the Protractor sensor
	protractor.begin(Wire,69);
	int protractorConnected = protractor.read(0); 

	pinMode(LED_PIN, OUTPUT);                		//set up the search light output pin on Arduino board
	mode = FINDVICTIM;                      		//set mission to Mode 0
	pixy.setServos(0, SEARCH);               		//set PixyCam at 90 degree for survivor search
	buzzer.playMode(PLAY_AUTOMATIC);         		//play sound at the start of the SAR mision
	//waitForButtonAndCountDown(true);       		//only used for testing purposes to delay mission
}

//void waitForButtonAndCountDown(bool restarting) { 
//	button.waitForButton(); 
//}

void loop(){  
	int speed;
	int32_t panError, tiltError;
        
        // Mode 0 Search for Survivor
	if (mode == FINDVICTIM){ 
		protractor.read();
		int leftMotorSpeed;
		int rightMotorSpeed; 
		unsigned int bestPath;
		if (protractor.pathCount() > 0) {                //calculate the angle value for the most safe path
			bestPath = protractor.pathAngle(0);      //when AMR encounters obstacles on its way
		} 
		else { 
			bestPath = 90;                           //when no obstacles detected move forward
		}                                                 
		leftMotorSpeed = FULL_SPEED;                     //set max values for motor speed and 
		rightMotorSpeed = FULL_SPEED;                    //map the best path angle value for left and right motors speed
		if (bestPath < 90)  {
			leftMotorSpeed = FULL_SPEED * bestPath / 90;   
		} 
		else if (bestPath > 90)  {
			rightMotorSpeed = FULL_SPEED * (180 - bestPath) / 90;
		}
    		motors.setSpeeds(leftMotorSpeed, rightMotorSpeed);
    		uint16_t blocks;blocks = pixy.getBlocks();       //stop AMR when the survivor IR emission is detected
	  	if (blocks) {                                    
			motors.setSpeeds(0, 0);                  
			digitalWrite(LED_PIN, HIGH);             //the search light will blink 
			delay(200);                              //when the survivor is found
			digitalWrite(LED_PIN, LOW); 
			pixy.setServos(0, RESCUE);               //set PixyCam in search for drone position (120 degree)
			mode = FINDDRONE;                        //set mission for the mode 1
			Serial.println("takeoff");               //send serial command to ESP32 module to remotely get the drone in the air
		} 
	}
        // Mode 1 Search for the Rescue drone
	if (mode == FINDDRONE){ 
		uint16_t blocks; 
		blocks = pixy.getBlocks();
		while (!pixy.getBlocks()){                       //calculate the AMR angle as it is spinning until the drone IR emission is detected
			// Every 10 ms take a sample from the gyro
			if(millis()-time > sampleTime){ 
				time = millis(); 
				gyro.read(); 
				rate=((int)gyro.g.z-dc_offset)/100;
 				if(rate >= noise || rate <= -noise) angle += ((double)(prev_rate+ rate) * sampleTime) / 2000;
       				prev_rate = rate; if (angle < 0) angle += 360; else if (angle >= 360) angle -= 360;
			} 
       			if (angle > 350) {                      //stop spinning AMR after a full rotation
				motors.setSpeeds(0, 0); 
				digitalWrite(LED_PIN, HIGH);    //the light search will blink after one full rotation
				delay(50); 
				digitalWrite(LED_PIN, LOW); 
				delay(5000); 
				angle = 0;
			}
			else {  
				motors.setSpeeds(-200, 200); //keep the AMR spinning for a full rotation until the drone in mid air is found
			} 
    		}
    		motors.setSpeeds(0, 0);      			// stop the AMR when the drone is located in mid air
    		digitalWrite(LED_PIN, HIGH); 			// the search light will blink when
		delay(100);                  			// the drone is on FOV of the PixyCam
		digitalWrite(LED_PIN, LOW);  			// for a moment and the code will
		Serial.println(angle);       			//Send angle at which the drone is found to ESP32 module 
		mode = LANDDRONE;            			// switch to the second mode of the mission 
  	} 
  	if (mode == LANDDRONE)	{
		while (pixy.getBlocks()){ 			//Adjust tilt position of the PixyCam for locking the drone position in mid air within camera FOV
			int p = 160 - pixy.blocks[0].x; 	
      			tiltError = pixy.blocks[0].y-Y_CENTER; 	
      			tiltLoop.update(tiltError); 		
      			pixy.setServos(0,tiltLoop.m_pos);
      			speed = p; 				
			if (p < 0 and p > -50) {                //when the drone position in camera FOV is not centered
				speed = 2*p - 100;              //adjust AMR speed values to spin the bot and readjust the Pixycam 
			}
     			Serial.println(p);			//Send position of drone to ESP32 module
      			//motors.setSpeeds(-speed, speed);       
                        if (speed > 0) {                        
				motors.setSpeeds(-SPEED, SPEED);//spin AMR CCW to center the drone position in PixyCam FOV
			} 
			if (speed < 0) {
				motors.setSpeeds(SPEED, -SPEED);//spin AMR CW to center the drone position in PixyCam FOV
			}                                       
                        delay(abs(speed));                      
      			digitalWrite(LED_PIN, HIGH);		//the search light will stay on longer when the drone is not centered in PixyCam FOV 
			delay(abs(speed));
			digitalWrite(LED_PIN, LOW);
	  	}
		motors.setSpeeds(0, 0);				//do not move AMR when the drone is not in PixyCam FOV - required further code adjustment (repeating Mode 2)                                    
  	}
}