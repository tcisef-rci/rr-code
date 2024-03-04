/* Pololu Zumo + Protractor
This is an example showing how to use the Protractor sensor with a Zumo mini sumo robot version 1.x from Pololu.
See (http://www.pololu.com) for details on the Zumo robot
See (http://www.will-moore.com/protractor/ProtractorAngleProximitySensor_UserGuide.pdf) for details on the Protractor sensor

Between turns, the Zumo normally drives in a straight line hoping to hit something. With a Protractor installed, 
the Zumo can see an opponent, turn towards it, and switch to full speed to maximize it's push.

This example was based off ZumoCollisionDetect.ino which can be found in the ZumoExamples folder of the Pololu 
Zumo library.
The Pololu Zumo library can be downloaded at (https://github.com/pololu/zumo-shield)
The Protractor library can be downloaded at (https://github.com/robogao/Protractor)
*/

#include <ZumoBuzzer.h>
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <avr/pgmspace.h>
#include <Wire.h>
#include <LSM303.h>
#include <Protractor.h>

#include <SPI.h>  
#include <Pixy.h>



#define X_CENTER    160L
#define Y_CENTER    100L
#define RCS_MIN_POS     0L
#define RCS_MAX_POS     1000L
#define RCS_CENTER_POS	((RCS_MAX_POS-RCS_MIN_POS)/2)



class ServoLoop
{
public:
	ServoLoop(int32_t proportionalGain, int32_t derivativeGain);

	void update(int32_t error);

	int32_t m_pos;
	int32_t m_prevError;
	int32_t m_proportionalGain;
	int32_t m_derivativeGain;
};

// ServoLoop Constructor
ServoLoop::ServoLoop(int32_t proportionalGain, int32_t derivativeGain)
{
	m_pos = RCS_CENTER_POS;
	m_proportionalGain = proportionalGain;
	m_derivativeGain = derivativeGain;
	m_prevError = 0x80000000L;
}

// ServoLoop Update 
// Calculates new output based on the measured
// error and the current state.
void ServoLoop::update(int32_t error)
{
	long int velocity;
	char buf[32];
	if (m_prevError!=0x80000000)
	{	
		velocity = (error*m_proportionalGain + (error - m_prevError)*m_derivativeGain)>>10;

		m_pos += velocity;
		if (m_pos>RCS_MAX_POS) 
		{
			m_pos = RCS_MAX_POS; 
		}
		else if (m_pos<RCS_MIN_POS) 
		{
			m_pos = RCS_MIN_POS;
		}
	}
	m_prevError = error;
}
// End Servo Loop Class
//---------------------------------------

Pixy pixy;  // Declare the camera object

ServoLoop panLoop(200, 200);  // Servo loop for pan
ServoLoop tiltLoop(150, 200); // Servo loop for tilt

//ZumoMotors motors;  // declare the motors on the zumo

//---------------------------------------
// Setup - runs once at startup
//---------------------------------------

uint32_t lastBlockTime = 0;








// #define LOG_SERIAL // write log output to serial port

#define LED 13
Pushbutton button(ZUMO_BUTTON); // pushbutton on pin 12

// Accelerometer Settings
#define RA_SIZE 3  // number of readings to include in running average of accelerometer readings
#define XY_ACCELERATION_THRESHOLD 2400  // for detection of contact (~16000 = magnitude of acceleration due to gravity)

// Reflectance Sensor Settings
#define NUM_SENSORS 6
unsigned int sensor_values[NUM_SENSORS];
// this might need to be tuned for different lighting conditions, surfaces, etc.
#define QTR_THRESHOLD  1500 // microseconds
ZumoReflectanceSensorArray sensors(QTR_NO_EMITTER_PIN);

// Motor Settings
ZumoMotors motors;

// these might need to be tuned for different motor types
#define REVERSE_SPEED     200 // 0 is stopped, 400 is full speed
#define TURN_SPEED        200
#define SEARCH_SPEED      200
#define SUSTAINED_SPEED   400 // switches to SUSTAINED_SPEED from FULL_SPEED after FULL_SPEED_DURATION_LIMIT ms
#define FULL_SPEED        400
#define STOP_DURATION     100 // ms
#define REVERSE_DURATION  200 // ms
#define TURN_DURATION     300 // ms

#define RIGHT 1
#define LEFT -1

enum ForwardSpeed { SearchSpeed, SustainedSpeed, FullSpeed };
ForwardSpeed _forwardSpeed;  // current forward speed setting
unsigned long full_speed_start_time;
#define FULL_SPEED_DURATION_LIMIT     250  // ms

// Sound Effects
ZumoBuzzer buzzer;
const char sound_effect[] PROGMEM = "O4 T100 V15 L4 MS g12>c12>e12>G6>E12 ML>G2"; // "charge" melody
 // use V0 to suppress sound effect; v15 for max volume

// Protractor Sensor
Protractor protractor;
 
 // Timing
unsigned long loop_start_time;
unsigned long last_turn_time;
unsigned long contact_made_time;
#define MIN_DELAY_AFTER_TURN          400  // ms = min delay before detecting contact event
#define MIN_DELAY_BETWEEN_CONTACTS   1000  // ms = min delay between detecting new contact event

// RunningAverage class 
// based on RunningAverage library for Arduino
// source:  http://playground.arduino.cc/Main/RunningAverage
template <typename T> 
class RunningAverage
{
  public:
    RunningAverage(void);
    RunningAverage(int);
    ~RunningAverage();
    void clear();
    void addValue(T);
    T getAverage() const;
    void fillValue(T, int);
  protected:
    int _size;
    int _cnt;
    int _idx;
    T _sum;
    T * _ar;
    static T zero;
};

// Accelerometer Class -- extends the LSM303 Library to support reading and averaging the x-y acceleration 
//   vectors from the onboard LSM303DLHC accelerometer/magnetometer
class Accelerometer : public LSM303
{
  typedef struct acc_data_xy
  {
    unsigned long timestamp;
    int x;
    int y;
    float dir;
  } acc_data_xy;
  
  public: 
    Accelerometer() : ra_x(RA_SIZE), ra_y(RA_SIZE) {};
    ~Accelerometer() {};
    void enable(void);
    void getLogHeader(void);
    void readAcceleration(unsigned long timestamp);
    float len_xy() const;
    float dir_xy() const;
    int x_avg(void) const;
    int y_avg(void) const;
    long ss_xy_avg(void) const;
    float dir_xy_avg(void) const;
  private:
    acc_data_xy last;
    RunningAverage<int> ra_x;
    RunningAverage<int> ra_y;   
};

Accelerometer lsm303;
boolean in_contact;  // set when accelerometer detects contact with opposing robot

// forward declaration
void setForwardSpeed(ForwardSpeed speed);

void setup()
{  
  // Initiate the Wire library and join the I2C bus as a master
  Wire.begin();
  
  // Initiate LSM303
  lsm303.init();
  lsm303.enable();

  // Initiate Protractor
  protractor.begin(Wire,69);
  int protractorConnected = protractor.read(0);
  
  
//#ifdef LOG_SERIAL

  //Serial.begin(9600);
	//Serial.print("Starting...\n");

	pixy.init();
  Serial.begin(9600);
  lsm303.getLogHeader();
  
  if(protractorConnected) {
    Serial.println("Protractor Connected");
  }else{
    Serial.println("Protractor not Connected");
  }
//#endif

  randomSeed((unsigned int) millis());
  
  // uncomment if necessary to correct motor directions
  //motors.flipLeftMotor(true);
  //motors.flipRightMotor(true);

  pinMode(LED, HIGH);
  buzzer.playMode(PLAY_AUTOMATIC);
  waitForButtonAndCountDown(false);
}

void waitForButtonAndCountDown(bool restarting)
{ 
#ifdef LOG_SERIAL
  Serial.print(restarting ? "Restarting Countdown" : "Starting Countdown");
  Serial.println();
#endif
  
  digitalWrite(LED, HIGH);
  button.waitForButton();
  digitalWrite(LED, LOW);
   
  // play audible countdown
  for (int i = 0; i < 3; i++)
  {
    delay(1000);
    buzzer.playNote(NOTE_G(3), 50, 12);
  }
  delay(1000);
  buzzer.playFromProgramSpace(sound_effect);
  delay(1000);
  
  // reset loop variables
  in_contact = false;  // 1 if contact made; 0 if no contact or contact lost
  contact_made_time = 0;
  last_turn_time = millis();  // prevents false contact detection on initial acceleration
  _forwardSpeed = SearchSpeed;
  full_speed_start_time = 0;
}





void loop()
{
  if (button.isPressed())
  {
    // if button is pressed, stop and wait for another press to go again
    motors.setSpeeds(0, 0);
    button.waitForRelease();
    waitForButtonAndCountDown(true);
 }

  protractor.read();
  uint16_t blocks;
	blocks = pixy.getBlocks();
  //Serial.println(blocks);  
  // If we have blocks in sight, track and follow them
	if (blocks)
  	{
	  	int trackedBlock = TrackBlock(blocks);
	  	//FollowBlock(trackedBlock);
	  	lastBlockTime = millis();
	  }  
	else if (millis() - lastBlockTime > 100)
  	{
	  	//motors.setLeftSpeed(0);
	  	//motors.setRightSpeed(0);
	  	//ScanForBlocks();
	  }                                                                 
  
  int leftMotorSpeed;
  int rightMotorSpeed;
  unsigned int bestPath;

  // Determine the angle of the best path so we can drive in that direction
  if (protractor.pathCount() > 0)
  {
    bestPath = protractor.pathAngle(0);
  }
  else
  {
    bestPath = 90;
  }

  // Drive in the direction of the path
  leftMotorSpeed = FULL_SPEED;
  rightMotorSpeed = FULL_SPEED;
  if (bestPath < 90)   // openPath of 0 to 89 means path is on the left, left motor should slow down and right motor full speed to turn left
  {
    leftMotorSpeed = FULL_SPEED * bestPath / 90;
  }
  else if (bestPath > 90)   // openPath of 91 to 180 means path is on the right, right motor should slow down and left motor full speed to turn right
  {
    rightMotorSpeed = FULL_SPEED * (180 - bestPath) / 90;
  }
  motors.setSpeeds(leftMotorSpeed, rightMotorSpeed);
}


// execute turn 
// direction:  RIGHT or LEFT
// randomize: to improve searching
void turn(char direction, bool randomize)
{
#ifdef LOG_SERIAL
  Serial.print("turning ...");
  Serial.println();
#endif

  // assume contact lost
  on_contact_lost();
  
  static unsigned int duration_increment = TURN_DURATION / 4;
  
  // motors.setSpeeds(0,0);
  // delay(STOP_DURATION);
  motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
  delay(REVERSE_DURATION);
  motors.setSpeeds(TURN_SPEED * direction, -TURN_SPEED * direction);
  delay(randomize ? TURN_DURATION + (random(8) - 2) * duration_increment : TURN_DURATION);
  int speed = getForwardSpeed();
  motors.setSpeeds(speed, speed);
  last_turn_time = millis();
}

void setForwardSpeed(ForwardSpeed speed)
{
  _forwardSpeed = speed;
  if (speed == FullSpeed) full_speed_start_time = loop_start_time;
}

int getForwardSpeed()
{
  int speed;
  switch (_forwardSpeed)
  {
    case FullSpeed:
      speed = FULL_SPEED;
      break;
    case SustainedSpeed:
      speed = SUSTAINED_SPEED;
      break;
    default:
      speed = SEARCH_SPEED;
      break;
  }
  return speed;
}
  
// check for contact, but ignore readings immediately after turning or losing contact
bool check_for_contact()
{
  static long threshold_squared = (long) XY_ACCELERATION_THRESHOLD * (long) XY_ACCELERATION_THRESHOLD;
  return (lsm303.ss_xy_avg() >  threshold_squared) && \
    (loop_start_time - last_turn_time > MIN_DELAY_AFTER_TURN) && \
    (loop_start_time - contact_made_time > MIN_DELAY_BETWEEN_CONTACTS);
}

// sound horn and accelerate on contact -- fight or flight
void on_contact_made()
{
#ifdef LOG_SERIAL
  Serial.print("contact made");
  Serial.println();
#endif
  in_contact = true;
  contact_made_time = loop_start_time;
  setForwardSpeed(FullSpeed);
  buzzer.playFromProgramSpace(sound_effect);
}

// reset forward speed
void on_contact_lost()
{
#ifdef LOG_SERIAL
  Serial.print("contact lost");
  Serial.println();
#endif
  in_contact = false;
  setForwardSpeed(SearchSpeed);
}

// class Accelerometer -- member function definitions

// enable accelerometer only
// to enable both accelerometer and magnetometer, call enableDefault() instead
void Accelerometer::enable(void)
{
  // Enable Accelerometer
  // 0x27 = 0b00100111
  // Normal power mode, all axes enabled
  writeAccReg(LSM303::CTRL_REG1_A, 0x27);

  if (getDeviceType() == LSM303::device_DLHC)
  writeAccReg(LSM303::CTRL_REG4_A, 0x08); // DLHC: enable high resolution mode
}

void Accelerometer::getLogHeader(void)
{
  Serial.print("millis    x      y     len     dir  | len_avg  dir_avg  |  avg_len");
  Serial.println();
}

void Accelerometer::readAcceleration(unsigned long timestamp)
{
  readAcc();
  if (a.x == last.x && a.y == last.y) return;
  
  last.timestamp = timestamp;
  last.x = a.x;
  last.y = a.y;
  
  ra_x.addValue(last.x);
  ra_y.addValue(last.y);
 
#ifdef LOG_SERIAL
 Serial.print(last.timestamp);
 Serial.print("  ");
 Serial.print(last.x);
 Serial.print("  ");
 Serial.print(last.y);
 Serial.print("  ");
 Serial.print(len_xy());
 Serial.print("  ");
 Serial.print(dir_xy());
 Serial.print("  |  ");
 Serial.print(sqrt(static_cast<float>(ss_xy_avg())));
 Serial.print("  ");
 Serial.print(dir_xy_avg());
 Serial.println();
#endif
}

float Accelerometer::len_xy() const
{
  return sqrt(last.x*a.x + last.y*a.y);
}

float Accelerometer::dir_xy() const
{
  return atan2(last.x, last.y) * 180.0 / M_PI;
}

int Accelerometer::x_avg(void) const
{
  return ra_x.getAverage();
}

int Accelerometer::y_avg(void) const
{
  return ra_y.getAverage();
}

long Accelerometer::ss_xy_avg(void) const
{
  long x_avg_long = static_cast<long>(x_avg());
  long y_avg_long = static_cast<long>(y_avg()); 
  return x_avg_long*x_avg_long + y_avg_long*y_avg_long;
}

float Accelerometer::dir_xy_avg(void) const
{
  return atan2(static_cast<float>(x_avg()), static_cast<float>(y_avg())) * 180.0 / M_PI;
}



// RunningAverage class 
// based on RunningAverage library for Arduino
// source:  http://playground.arduino.cc/Main/RunningAverage
// author:  Rob.Tillart@gmail.com
// Released to the public domain

template <typename T>
T RunningAverage<T>::zero = static_cast<T>(0);

template <typename T>
RunningAverage<T>::RunningAverage(int n)
{
  _size = n;
  _ar = (T*) malloc(_size * sizeof(T));
  clear();
}

template <typename T>
RunningAverage<T>::~RunningAverage()
{
  free(_ar);
}

// resets all counters
template <typename T>
void RunningAverage<T>::clear() 
{ 
  _cnt = 0;
  _idx = 0;
  _sum = zero;
  for (int i = 0; i< _size; i++) _ar[i] = zero;  // needed to keep addValue simple
}

// adds a new value to the data-set
template <typename T>
void RunningAverage<T>::addValue(T f)
{
  _sum -= _ar[_idx];
  _ar[_idx] = f;
  _sum += _ar[_idx];
  _idx++;
  if (_idx == _size) _idx = 0;  // faster than %
  if (_cnt < _size) _cnt++;
}

// returns the average of the data-set added so far
template <typename T>
T RunningAverage<T>::getAverage() const
{
  if (_cnt == 0) return zero; // NaN ?  math.h
  return _sum / _cnt;
}

// fill the average with a value
// the param number determines how often value is added (weight)
// number should preferably be between 1 and size
template <typename T>
void RunningAverage<T>::fillValue(T value, int number)
{
  clear();
  for (int i = 0; i < number; i++) 
  {
    addValue(value);
  }
}

int oldX, oldY, oldSignature;

//---------------------------------------
// Track blocks via the Pixy pan/tilt mech
// (based in part on Pixy CMUcam5 pantilt example)
//---------------------------------------
int TrackBlock(int blockCount)
{
	int trackedBlock = 0;
	long maxSize = 0;

	Serial.print("blocks =");
	Serial.println(blockCount);

	for (int i = 0; i < blockCount; i++)
	{
		if ((oldSignature == 0) || (pixy.blocks[i].signature == oldSignature))
		{
			long newSize = pixy.blocks[i].height * pixy.blocks[i].width;
      Serial.println(newSize);
			if (newSize > maxSize)
			{
				trackedBlock = i;
				maxSize = newSize;
			}
		}
	}

	int32_t panError = X_CENTER - pixy.blocks[trackedBlock].x;
  Serial.println(panError);  
	int32_t tiltError = pixy.blocks[trackedBlock].y - Y_CENTER;
  Serial.println(tiltError);

	panLoop.update(panError);
	tiltLoop.update(tiltError);

	//pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);

	oldX = pixy.blocks[trackedBlock].x;
	oldY = pixy.blocks[trackedBlock].y;
	oldSignature = pixy.blocks[trackedBlock].signature;
	return trackedBlock;
}

//---------------------------------------
// Follow blocks via the Zumo robot drive
//
// This code makes the robot base turn 
// and move to follow the pan/tilt tracking
// of the head.
//---------------------------------------
int32_t size = 400;
void FollowBlock(int trackedBlock)
{
	int32_t followError = RCS_CENTER_POS - panLoop.m_pos;  // How far off-center are we looking now?

	// Size is the area of the object.
	// We keep a running average of the last 8.
	size += pixy.blocks[trackedBlock].width * pixy.blocks[trackedBlock].height; 
	size -= size >> 3;

	// Forward speed decreases as we approach the object (size is larger)
	int forwardSpeed = constrain(400 - (size/256), -100, 400);  

	// Steering differential is proportional to the error times the forward speed
	int32_t differential = (followError + (followError * forwardSpeed))>>8;

	// Adjust the left and right speeds by the steering differential.
	int leftSpeed = constrain(forwardSpeed + differential, -400, 400);
	int rightSpeed = constrain(forwardSpeed - differential, -400, 400);

	// And set the motor speeds
	motors.setLeftSpeed(leftSpeed);
	motors.setRightSpeed(rightSpeed);
}

//---------------------------------------
// Random search for blocks
//
// This code pans back and forth at random
// until a block is detected
//---------------------------------------
int scanIncrement = (RCS_MAX_POS - RCS_MIN_POS) / 150;
uint32_t lastMove = 0;

void ScanForBlocks()
{
	if (millis() - lastMove > 20)
	{
		lastMove = millis();
		panLoop.m_pos += scanIncrement;
		if ((panLoop.m_pos >= RCS_MAX_POS)||(panLoop.m_pos <= RCS_MIN_POS))
		{
			tiltLoop.m_pos = random(RCS_MAX_POS * 0.6, RCS_MAX_POS);
			scanIncrement = -scanIncrement;
			if (scanIncrement < 0)
			{
				motors.setLeftSpeed(-250);
				motors.setRightSpeed(250);
			}
			else
			{
				motors.setLeftSpeed(+180);
				motors.setRightSpeed(-180);
			}
			delay(random(250, 500));
		}

		//pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
	}
}
