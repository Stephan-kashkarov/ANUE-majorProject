/* ANU MAZERUNNER ROBOT CODE
	this code was written for ANUE
	This file contains all the classes
	used in the program and is run 
	first by default
	by: Stephan Kashkarov 
	in: November 2018
*/
// imports
#include <Servo.h>
#include <SoftwareSerial.h>


// Functions

void pause(int len)
/*

	pause 

	This function is an alternative to the default delay function
	as that it dosen't interfear with the servo library or
	other PWM signals.

	@param int len ~ the lenght of the pause in milliseconds

*/
{
	unsigned long timer = millis();
	while (timer + len >= millis())
	{
		if (timer + len < millis())
		{
			break;
		}
	}
}

byte arrMinIndex(int *arr, byte size)
{
	byte minimum = 0;
	for (size_t i = 0; i < size; ++i)
	{
		if (arr[i] < arr[minimum] && arr[i] > 3)
		{
			minimum = i;
		}
	}
	return minimum;
}

byte limitedArrMinIndex(int* arr, byte start, byte finish)
{
	byte minimum = start;
	for (size_t i = start; i < finish; ++i)
	{
		if (arr[i] < arr[minimum] && arr[i] > 3)
		{
			minimum = i;
		}
	}
	return minimum;
}

// Classes

class Sensor
/*

	Class Sensor

	This class is used to organise the main
	sensor of the robot, The ultrasonic - Servo
	combo.

	This class handles servo movment and ultrasonic
	scanning.
*/
{
  private:
	/*

			The variables of Servo

			The variables of servo include:
				-> servo | an instance of the servo class
						| running on the servo pin
				-> trig  | The trig pin of the ultrasonic
				-> echo  | The echo pin of the ultrasonic
		*/
	Servo servo;
	byte trig;
	byte echo;

  public:
	/*
			The functions of Servo
			
			The functions of Servo include:
				-> Sensor(byte pins[3])                 | The initaliser of Servo
				-> void moveServo(byte degree)          | Moves the servo to the specified degree
				-> int checkSonarSmart(byte degree)    | Checks distance at degree
				-> int checkSonarDumb()                | Checks current distance w/o servo
				-> void forwardSweep(int *points[5])   | Does a quick 5 point sweep
				-> void fullSweep(int *distances[180]) | Does a full 179 point sweep

			all functions are defined below
			the class definition
		*/
	Sensor(byte* pins);
	void moveServo(byte degree);
	int pulseUltra();
	int checkSonarSmart(byte degree);
	int checkSonarDumb();
	void forwardSweep(int *points);
	void fullSweep(int *distances, byte start, byte finish);
};

// Functions of Sensor
Sensor::Sensor(byte* pins)
/*
	Sensor::Sensor

	Initaliser of Sensor class.
	
	Takes an array of pins and 
	sets up all connections.

	@param(byte pins[3]) ~ array of 3 pins { servo, trig, echo }
*/
{
	// class variable binding
	servo.attach(pins[0]); // attaches servo to pin servo
	trig = pins[1];
	echo = pins[2];

	// pinmode declarations
	pinMode(pins[1], OUTPUT);
	pinMode(pins[2], INPUT);
}

void Sensor::moveServo(byte degree)
/*
	Sensor::moveServo

	moving servo function

	This function moves the servo to specified angle
	and stops it there

	@param (byte degree) ~ a number between 0 and 179 that the servo will move to

	//TODO make work with attach and detach for smooth operation
*/
{
	// ensure movement untill completion
	while (servo.read() != degree)
	{
		// tells servo to move to degree
		servo.write(degree);
	}
	// takes a quick break
	pause(1);
}

int Sensor::pulseUltra()
/*
	Sensor::pulseUltra

	the simple ultrasonic operation

	returns: int distance ~ The distance that was
	                       ~ measured with the ultrasonic in cms
*/
{
	// local variables
	int distance;

	//resets the trig pin just incase
	digitalWrite(trig, LOW);
	pause(1);
	// prepairs trigger
	digitalWrite(trig, HIGH);
	pause(1);
	// activates trigger
	digitalWrite(trig, LOW);
	// reads length of pulse
	distance = pulseIn(echo, HIGH);

	// does arethmetic for distance
	return distance / 29 / 2;
}

int Sensor::checkSonarDumb()
/*
	Sensor::checkSonarDumb

	checks distance at current action

	This function takes three measurments of the ultrasonic
	at the current angle and returns the avg distance.

	@param (byte degree) ~ a number between 0 and 179 that the servo will move to

	returns: int distance ~ The distance that was
	                       ~ measured with the ultrasonic in cms

*/
{
	// local variables
	pause(100);
	int avgDistance;

	// takes avg distance
	avgDistance = pulseUltra();
	// avgs it 3 more times
	for (size_t i = 0; i < 3; ++i)
	{
		avgDistance += pulseUltra();
		avgDistance /= 2;
	}
	return avgDistance;
}

int Sensor::checkSonarSmart(byte degree)
/*
	Sensor::checkSonarSmart

	checks distance at specified angle

	This function moves the servo to specified angle
	returns the average distance the ultrasonic mesured
	at said	distance.

	@param (byte degree) ~ a number between 0 and 179 that the servo will move to

	returns: int distance ~ The distance that was
	                       ~ measured with the ultrasonic in cms
*/
{
	// moves servo to degree
	pause(10);
	moveServo(degree);
	pause(100);
	// returns the avg distance
	return checkSonarDumb();
}

void Sensor::forwardSweep(int *points)
/*
	Sensor::forwardSweep

	checks 5 important points

	This function goes over the five important points in
	forwards navigation these include 45, 67, 90, 113, 135
	this then modifes the list given as input.

	returns: int *points ~ modifies list given as input with
	                      ~ distances in order.
*/
{
	// local variables
	byte angles[5] = {45, 67, 90, 113, 135};

	// iterates throgh angles
	for (size_t i = 0; i < 5; ++i)
	{
		pause(100);
		points[i] = checkSonarSmart(angles[i]);
	}
}

void Sensor::fullSweep(int* distances, byte start, byte finish)
/*
	Sensor::fullSweep

	does a full 180 degree sweep

	This function goes through all 180 points
	of rotation in the servo and modifies list
	given as input with these values.

	returns: int *distances ~ modifies list given as input with
	                         ~ distances in order.
*/
{
	// iterates throgh all angles
	for (size_t i = start; i < finish; ++i)
	{
		distances[i] = checkSonarSmart(i);
		pause(5);
	}
}

class Motors
/*

	Class Motors

	This class controls the operations for all the motors
	it controls all directional and pwm based movement.
*/
{
	private:
		/*
			Variables of Motors

			the Variables of Motors include:
				-> motorPin1A | The motor pin left top
				-> motorPin1B | The motor pin left bottom
				-> motorPin2A | The motor pin right top
				-> motorPin2B | The motor pin right bottom
		*/
		byte motorPin1A;
		byte motorPin1B;
		byte motorPin2A;
		byte motorPin2B;

	public:
	/*
			Functions of Motors

			the functions of motors include:
				-> Motors(byte* motorPins) | Initaliser of Motors
				-> void forward(byte percent) | Forward function moves forward and scans
				-> void left()   | Left function turns left on spot
				-> void right()  | Right function turns right on spot
				-> void back(byte percent)   | Back function moves back
				-> void stop()               | Stop function stops the motors
		*/
		Motors(byte* motorPins);
		void pinModeReset();
		void forward();
		void left();
		void right();
		void back();
		void stop();
};

// Functions of Motors
Motors::Motors(byte* motorPins)
/*
	Motors::Motors

	Initaliser of Motors

	takes in an array of pins of length 4
	these pins are then mapped to the 
	local variables.
*/
{
	// sets pinmodes on all pins
	for (size_t i = 0; i < 4; ++i)
	{
		pinMode(motorPins[i], OUTPUT);
	}

	// binds pins to locals
	motorPin1A = motorPins[0];
	motorPin1B = motorPins[1];
	motorPin2A = motorPins[2];
	motorPin2B = motorPins[3];

}

void Motors::pinModeReset()
/*
	Motors::pinModeReset

	resets the pinModes

	This function resets the pinmodes of all motor pins.
	It is used after a pin was used for pwm to ensure that
	the pin can handle normal digital signals again
*/
{
	// Sets pinmodes
	pinMode(motorPin1A, OUTPUT);
	pinMode(motorPin1B, OUTPUT);
	pinMode(motorPin2A, OUTPUT);
	pinMode(motorPin2B, OUTPUT);
}

void Motors::forward()
/*
	Motors::forward

	moves the robot forward

	moves the robot forward at a percentage of full power
	as specified by input. 

	@param (byte percent) ~ a value between 0 and 100 which
	                      ~ translates into a pwm value.
*/
{
	pinModeReset();
	analogWrite(motorPin1A, 255 - 140);
	digitalWrite(motorPin1B, LOW);
	digitalWrite(motorPin2A, HIGH);
	digitalWrite(motorPin2B, LOW);
}

void Motors::left()
/*
	Motors::left

	Turns the robot left.

	This function turns the robot left at specified
	percentage of full speed.

	@param (byte percent) ~ a value between 0 and 100 which
	                      ~ translates into a pwm value.
*/
{
	// PWMs to the PWM pins and sets others low
	pinModeReset();
	digitalWrite(motorPin1A, LOW);
	digitalWrite(motorPin1B, HIGH);
	digitalWrite(motorPin2A, HIGH);
	digitalWrite(motorPin2B, LOW);
}

void Motors::right()
/*
	Motors::right

	Turns the robot right.

	This function turns the robot right at specified
	percentage of full speed.

	@param (byte percent) ~ a value between 0 and 100 which
	                      ~ translates into a pwm value.
*/
{
	pinModeReset();
	digitalWrite(motorPin1A, HIGH);
	digitalWrite(motorPin1B, LOW);
	digitalWrite(motorPin2A, LOW);
	digitalWrite(motorPin2B, HIGH);
}

void Motors::back()
/*
	Motors::back

	Turns the robot back.

	This function moves the robot back at specified
	percentage of full speed.

	@param (byte percent) ~ a value between 0 and 100 which
	                      ~ translates into a pwm value.
*/
{
	pinModeReset();
	analogWrite(motorPin1A, (255 / 2) - 75);
	digitalWrite(motorPin1B, HIGH);
	analogWrite(motorPin2A, 255 / 2);
	digitalWrite(motorPin2B, HIGH);
}

void Motors::stop()
/*
	Motors::stop

	stops the robot

	This function stops the robot from doing any movement
*/
{
	pinModeReset();
	digitalWrite(motorPin1A, LOW);
	digitalWrite(motorPin1B, LOW);
	digitalWrite(motorPin2A, LOW);
	digitalWrite(motorPin2B, LOW);
}

class Robot
/*
	Class Robot

	This class ties together the whole robot
	it contains all the high level logic assosiated
	with the robot and also houses the communication
	layer.
*/
{
	private:
		/*
			Variables of Robot

			The variables of Robot include:
				-> sensor         | This variable contains an instance of the servo class
				-> motors         | This variable contains an instance of the motor class
				-> bluetooth      | This variable contains an instance of the SoftwareSerial class
				-> distances[180] | This is an array of 180 points around the robot
				-> points[5]      | This variable contains 5 important points for quick scanning
		*/
	  Motors* motors = NULL;
	  Sensor* sensor = NULL;
	  SoftwareSerial *bluetooth = NULL;
	  int distances[180];
	  int points[5];

	public:
		/*
			Functions of Robot

			The functions of Robot include:
				-> void init()                        | The pinmode setter for void setup
				-> void bluetooth()                   | Bluetooth/Serial reading protocal
				-> void pathfinding()                 | Pathfinding algorithm
				-> void remote()                      | Bluetooth/Serial based remote control system
		*/
		void init(byte *motorPins, byte *sensorPins, byte *comPins);
		int readBluetooth();
		void pathfinding();
		void remote();
};

void Robot::init(byte *motorPins, byte *sensorPins, byte *comPins)
/*
	Robot::init

	void setup based initaliser

	This method is to be run within void setup to
	define pinmodes and initalise all the external
	classes within Robot
*/
{
	// initalises all internal classes
	motors = new Motors(motorPins);
	sensor = new Sensor(sensorPins);
	bluetooth = new SoftwareSerial(comPins[0], comPins[1]);

	// Starts up bluetooth
	bluetooth->begin(9600);

	// zeros ultrasonic
	sensor->moveServo(90);
}

int Robot::readBluetooth()
/*
	Robot::bluetooth

	This is the input protocal

	This function reads bluetooth and returns a response
*/
{
	int input;
	bluetooth->println("Waiting for Input!");
	while (true)
	{
		// if incomming data
		if (bluetooth->available() > 0)
		{
			input = bluetooth->read();
			bluetooth->println("Input Recieved: ");
			bluetooth->println(input);
			while(bluetooth->available() >0)
			{
				bluetooth->read();
			}
			return input;
		}
	}
}

void Robot::pathfinding()
/**/
{
	byte maxSize = 180;
	byte minDegree;
	bool left;
	while(true)
	{
		bluetooth->println("Starting pathfinding");
		sensor->fullSweep(distances, 0, 180);
		sensor->moveServo(90);
		if (distances[90] > 7)
		{
			bluetooth->println("Forward clear");
			motors->forward();
			while (true)
			{
				if (sensor->checkSonarSmart(90) < 7)
				{
					bluetooth->println("Obsticle forward");
					break;
				}
				else if (sensor->checkSonarSmart(10) < 7)
				{
					bluetooth->println("Obsticle right");
					motors->left();
					pause(100);
					motors->stop();
				}
				else if (sensor->checkSonarSmart(170) < 7)
				{
					bluetooth->println("Obsticle left");
					motors->right();
					pause(100);
					motors->stop();
				}
			}
			motors->stop();
		}
		else
		{
			bluetooth->println("Forward blocked");
			left = false;

			minDegree = arrMinIndex(distances, maxSize);
			if (distances[170] > distances[10])
			{
				bluetooth->println("Turning left");
				left = true;
			}
			else
			{
				bluetooth->println("Turning right");
				left = false;
			}
			while (minDegree > 10 || minDegree < 170)
			{
				bluetooth->print("Current min degree is ");
				bluetooth->println(minDegree);
				if (left)
				{
					motors->left();
				}
				else
				{
					motors->right();
				}
				sensor->fullSweep(distances, minDegree - 2, minDegree + 2);
				minDegree = limitedArrMinIndex(distances, minDegree - 2, minDegree + 2);
			}
			bluetooth->println("Obsticle avoided");
			motors->stop();
		}
	}
}

void Robot::remote()
/*
	Robot::remoteControl

	remote control method

	this takes an input and excecutes then askes for a new input

	Possible inputs:
		-> 0  ~ move Forward
		-> 1  ~ move Back
		-> 2  ~ turn Left
		-> 3  ~ turn Right
		-> 4  ~ enter pathfinding mode
		-> 5  ~ Scan 180 degrees
		-> 6  ~ force stop
*/
{
	// initalises state
	byte state = 9;

	// Starts control loop
	while (true)
	{
		// get input
		state = readBluetooth();
		switch (state)
		{
			case 0:
				// forward state
				bluetooth->println("Robot: Moving Forward");
				// Checks surroundings before starting
				sensor->forwardSweep(points);
				if (points[0] < 4 || points[4] < 4 || points[1] < 4 || points[3] < 4 || points[2] < 5)
				{
					break;
				}
				// sets motors to full forward
				motors->forward();
				while(true)
				{
					// checks 5 points
					sensor->forwardSweep(points);
					// break cases
					if (points[0] < 5 || points[4] < 5 || points[1] < 7 || points[3] < 7 || points[2] < 10 || bluetooth->available() > 0)
					{
						bluetooth->println("Exiting forward mode");
						bluetooth->println(bluetooth->read());
						// kills forward
						motors->stop();
						break;
					}
				}
				// resets state
				state = 9;
				break;
			case 1:
				// Back state
				bluetooth->println("Robot: Moving Back");
				// moves back at 50%
				motors->back();
				// resets state
				state = 9;
				break;
			case 2:
				// left state
				bluetooth->println("Robot: Moving Left");
				// moves left at 50%
				motors->left();
				// resets state
				state = 9;
				break;
			case 3:
				// Right state
				bluetooth->println("Robot: Moving Right");
				// moves right at 50%
				motors->right();
				// resets state
				state = 9;
				break;
			case 4:
				// pathfinding state
				bluetooth->println("Robot: Entering Pathfinding");
				pathfinding();
				// resets state
				state = 9;
				break;
			case 5:
				// scanning state
				bluetooth->println("Robot: Scanning");
				// updates distances
				sensor->fullSweep(distances, 0, 180);
				pause(1);
				// interates through distances
				for(size_t i = 0; i < 179; ++i)
				{
					// format->prints the value
					bluetooth->print("distance at ");
					bluetooth->print(i);
					bluetooth->print(" degrees is ");
					bluetooth->println(distances[i]);
				}
				// resets servo
				sensor->moveServo(90);
				// resets state
				state = 9;
				break;
			case 6:
				bluetooth->println("Robot: Stopping");
				motors->stop();
				sensor->moveServo(90);
				// resets state
				state = 9;
				break;
			
			default:
				// resets state
				state = 9;
				break;
		}
	}
}

// Variable definition
byte definedMotors[4] = {5, 8, 6, 11}; // motors
byte definedMisc[3] = {2, 3, 4};       // servo, trig, echo
byte comPins[2] = {13, 12};            // bluetooth

// Creates instance of Robot named mike
// Robot mike;

// Sensor* melvin = NULL;
// Motors* greg = NULL;
Robot paul;

// Sensor(byte *pins);
	// void moveServo(byte degree);
	// int pulseUltra();
	// int checkSonarSmart(byte degree);
	// int checkSonarDumb();
	// void forwardSweep(int *points);
	// void fullSweep(int *distances);

// Motors(byte *motorPins);
	// void pinModeReset();
	// void forward(byte percent);
	// void left();
	// void right();
	// void back(byte percent);
	// void stop();

void setup()
{
	paul.init(definedMotors, definedMisc, comPins);
	// melvin = new Sensor(definedMisc);
	// greg = new Motors(definedMotors);
	// Serial.begin(9600);
}

void loop()
{
	paul.remote();
}

