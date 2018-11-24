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
	as that it dosen't interfear with the servo library.

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
/**/
{
}

void Sensor::fullSweep(long &distances)
/**/
{
}

class Motors
/*

*/
{
	private:
		/*

		*/
		byte motorPin1A;
		byte motorPin1B;
		byte motorPin2A;
		byte motorPin2B;

	public:
		/*

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
/**/
{
}

int Motors::forward(byte precent)
/**/
{
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
