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
	while (millis() + len >= millis())
	{
		if (millis() + len < millis())
		{
			break;
		}
	}
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
			-> Sensor(byte pins[3])              | The initaliser of Servo
			-> void moveServo(byte degree)       | Moves the servo to the specified degree
			-> long checkSonarSmart(byte degree) | Checks distance at degree
			-> long checkSonarDumb()             | Checks current distance w/o servo
			-> void forwardSweep(long &points)   | Does a quick 5 point sweep
			-> void fullSweep(long &distances)   | Does a full 179 point sweep

		all functions are defined below
		the class definition
	*/
	Sensor(byte pins[3]);
	void moveServo(byte degree);
	long Sensor::pulse();
	long checkSonarSmart(byte degree);
	long checkSonarDumb();
	void forwardSweep(long &points);
	void fullSweep(long &distances);
};

// Functions of Sensor
Sensor::Sensor(byte pins[3])
/*
	Sensor::Sensor

	Initaliser of Sensor class.
	
	Takes an array of pins and 
	sets up all connections.

	@param(byte pins[3]) ~ array of 3 pins { servo, trig, echo }
*/
{
	// class variable binding
	this->servo.attach(pins[0]); // attaches servo to pin servo
	this->trig = pins[1];
	this->echo = pins[2];

	// pinmode declarations
	pinMode(pin[1], OUTPUT);
	pinMode(pin[2], INPUT);
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
	while(this->servo.read() != degree)
	{
		// tells servo to move to degree
		this->servo.write(degree);
	}
	// takes a quick break
	pause(1);
}

long Sensor::pulse()
/*
	Sensor::pulse

	the simple ultrasonic operation

	returns: long distance ~ The distance that was
	                       ~ measured with the ultrasonic in cms
*/
{
	// local variables
	long distance;

	//resets the trig pin just incase
	digitalWrite(this->trig, LOW);
	pause(1);
	// prepairs trigger
	digitalWrite(this->trig, HIGH);
	pause(1);
	// activates trigger
	digitalWrite(this->trig, LOW);
	// reads length of pulse
	distance = pulseIn(this->echo, HIGH);

	// does arethmetic for distance
	return distance/29/2;
	
}

long Sensor::checkSonarDumb()
/*
	Sensor::checkSonarDumb

	checks distance at current action

	This function takes three measurments of the ultrasonic
	at the current angle and returns the avg distance.

	@param (byte degree) ~ a number between 0 and 179 that the servo will move to

	returns: long distance ~ The distance that was
	                       ~ measured with the ultrasonic in cms

*/
{
	// local variables
	long avgDistance;

	avgDistance = this->pulse();
	for (size_t i = 0; i < 3; ++i)
	{
		avgDistance += this->pulse();
		avgDistance/2;
	}
	return avgDistance;
}

long Sensor::checkSonarSmart(byte degree)
/*
	Sensor::checkSonarSmart

	checks distance at specified angle

	This function moves the servo to specified angle
	returns the average distance the ultrasonic mesured
	at said	distance.

	@param (byte degree) ~ a number between 0 and 179 that the servo will move to

	returns: long distance ~ The distance that was
	                       ~ measured with the ultrasonic in cms
*/
{
	// moves servo to degree
	this->moveServo(degree);
	// returns the avg distance
	return this->checkSonarDumb();
}


void Sensor::forwardSweep(long &points)
/*
	Sensor::forwardSweep

	checks 5 important points

	This function goes over the five important points in
	forwards navigation these include 0, 45, 90, 135, 180
	this then modifes the list given as input.

	returns: long &points ~ modifies list given as input with
	                      ~ distances in order.
*/
{
	// local variables
	byte angles[5] = {0, 45, 90, 135, 180};
	
	// iterates throgh angles
	for (size_t i = 0, i < 5; ++i)
	{
		points[i] = this->checkSonarSmart(angles[i]);
	}
}

void Sensor::fullSweep(long &distances)
/*
	Sensor::fullSweep

	does a full 180 degree sweep

	This function goes through all 180 points
	of rotation in the servo and modifies list
	given as input with these values.

	returns: long &distances ~ modifies list given as input with
	                         ~ distances in order.
*/
{
	// iterates throgh all angles
	for (size_t i = 0, i < 179; ++i)
	{
		distances[i] = this->checkSonarSmart(angles[i]);
		pause(1);
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
				-> Motors(byte motorPins[4]) | Initaliser of Motors
				-> int forward(byte precent) | Forward function moves forward and scans
				-> void left(byte precent)   | Left function turns left on spot
				-> void right(byte precent)  | Right function turns right on spot
				-> void back(byte precent)   | Back function moves back
				-> void stop()               | Stop function stops the motors
		*/
	  Motors(byte motorPins[4]);
	  int forward(byte precent);
	  void left(byte precent);
	  void right(byte precent);
	  void back(byte precent);
	  void stop();
};

// Functions of Motors
Motors::Motors(byte motorPins[4])
/*
	Motors::Motors

	Initaliser of Motors

	takes in an array of pins of length 4
	these pins are then mapped to the 
	local variables.
*/
{
	// binds pins to locals
	this->motorPin1A = motorPins[0];
	this->motorPin1B = motorPins[1];
	this->motorPin2A = motorPins[2];
	this->motorPin2B = motorPins[3];

	// sets pinmodes on all pins
	for (size_t i = 0; i < 4; ++i)
	{
		pinMode(motorPins[i], OUTPUT);
	}
}

int Motors::forward(byte precent)
/*
	Motors::forward

	forward function

	runs the car forward whilst scanning for obsticles
	this function can be run at a slower speed through pwm

	@param (byte percent) ~ a value between 0 and 100 which
	                      ~ translates into a pwm value.
*/
{
	if (percent == 100) // if full front speed
	{
		digitalWrite(motorPin1A, HIGH);
		digitalWrite(motorPin1B, LOW);
		digitalWrite(motorPin2A, HIGH);
		digitalWrite(motorPin2B, LOW);
	}
	else
	{
		// maps the percent to the PWM range
		percent = map(percent, 0, 100, 0, 255);
		// PWMs to the PWM pins and sets others low
		analogWrite(motorPin1A, percent);
		digitalWrite(motorPin1B, LOW);
		analogWrite(motorPin2A, percent);
		digitalWrite(motorPin2B, LOW);
	}
	// TODO: The scanning
}

void Motors::left(byte precent)
/**/
{
}

void Motors::right(byte precent)
/**/
{
}

void Motors::back(byte precent)
/**/
{
}

void Motors::stop()
/**/
{
}

class Robot
/*

*/
{
	private:
		/*
			
		*/
		Motors motors;
		Sensor sensor;
		long distances[180];
		long points[5];

	public:
		/*
			
		*/
		Robot();
		void init();
		void bluetooth();
		void pathfinding();
		void remote();
};

Robot::Robot()
/**/
{
}

void Robot::init()
/**/
{
}

void Robot::bluetooth()
/**/
{
}

void Robot::pathfinding()
/**/
{
}

void Robot::remote()
/**/
{
}
