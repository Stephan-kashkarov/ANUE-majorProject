/* ANU MAZERUNNER ROBOT CODE
	this code was written for ANUE
	by: Stephan Kashkarov 
	in: October 2018
*/
// debug
#define private public

// imports
#include <Servo.h>
#include <SoftwareSerial.h>

// Variable definition
byte definedMotors[4] = {8, 9, 10, 11}; // motors 
byte definedMisc[3] = {3, 4, 5}; // servo, trig, echo
SoftwareSerial bluetooth(12, 13); // bluetooth

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

class Robot
/*
	Class Robot

	This class contains all of the functions for the robot
	Its variables are:
		
		-> servoPin  | The pin related to the servo
		-> servo     | An instance of the servo class
		-> trig      | The trig pin of the ultrasonic sensor
		-> echo      | The echo pin of the ultrasonic sensor
		-> motorPins | An array contaning all the motor motorPins
					 | Ordered: Top-left Bottom-left, Top-right, Bottom-right
		-> distances | An array cantaining 180 different distance points
	*/
{
	private:
		// variables
		Servo servo;
		byte servoPin;
		byte trig;
		byte echo;
		byte motorPins[4];
		int distances [180];

	public:
		// function of robot
		void init(byte motors[4], byte misc[3]);
		void stop();
		void left();
		void right();
		void back();
		void forward();
		void moveServo(byte degree);
		unsigned int pulse();
		void scan();
		int checkBluetooth();
		void remoteControl();
		void printDistances();

};

void Robot::init(byte motors[4], byte misc[3])
/*
	Robot::init

	This function initializes all the pins in the robot

	@param: byte motors[4] ~ This is an array of the pins of the motors
							~ the pins should be orderd like should
							~ motor 1 a, motor 1 b, motor 2 a, motor 2 b 

	@param: byte misc[3]   ~ This is an array of the misc pins
							~ it contains the following pins in orderd
							~ servo pin, trig pin, echo pin

*/
{
	// declaring internal variables
	this->motorPins[0] = motors[0];
	this->motorPins[1] = motors[1];
	this->motorPins[2] = motors[2];
	this->motorPins[3] = motors[3];
	this->servoPin = misc[0];
	this->trig = misc[1];
	this->echo = misc[2];
	this->servo.attach(misc[0]); // attaches servo to servo pin

	for (byte i = 0; i < 4; ++i)
	{
		pinMode(this->motorPins[i], OUTPUT); // sets all pins to outputs
	}

	// more pinmodes
	pinMode(misc[0], OUTPUT);
	pinMode(this->trig, OUTPUT);
	pinMode(this->echo, INPUT);
}

void Robot::stop()
/*
	Robot::stop

	This function stops the robot

*/
{
	digitalWrite(this->motorPins[0], LOW);
	digitalWrite(this->motorPins[1], LOW);
	digitalWrite(this->motorPins[2], LOW);
	digitalWrite(this->motorPins[3], LOW);
}

void Robot::left()
/*
	Robot::left

	This function turns left

*/
{
	digitalWrite(this->motorPins[0], HIGH);
	digitalWrite(this->motorPins[1], LOW);
	digitalWrite(this->motorPins[2], LOW);
	digitalWrite(this->motorPins[3], HIGH);
}

void Robot::right()
/*
	Robot::right

	This function turns right

*/
{
	digitalWrite(this->motorPins[0], LOW);
	digitalWrite(this->motorPins[1], HIGH);
	digitalWrite(this->motorPins[2], HIGH);
	digitalWrite(this->motorPins[3], LOW);
}

void Robot::back()
/*
	Robot::back

	This function turns back while sensing

*/
{
	digitalWrite(this->motorPins[0], LOW);
	digitalWrite(this->motorPins[1], LOW);
	digitalWrite(this->motorPins[2], HIGH);
	digitalWrite(this->motorPins[3], HIGH);
}

void Robot::forward()
/*
	Robot::forward

	This function turns forward while sensing
*/
{
	int scan;
	int scanl;
	int scanr;
	digitalWrite(this->motorPins[0], HIGH);
	digitalWrite(this->motorPins[1], HIGH);
	digitalWrite(this->motorPins[2], LOW);
	digitalWrite(this->motorPins[3], LOW);
	bluetooth.println("Enter anything to break");
	// emptys bluetooth
	while (bluetooth.available() > 0)
	{
		bluetooth.read();
	}
	// scans for obsticles
	while (true)
	{
		// scan = this->pulse();
		// bluetooth.print("Forward Scan distance: ");
		// bluetooth.println(scan);
		// if (scan < 10) // Breaks by distance
		// {
		// 	bluetooth.println("Movement Obstructed");
		// 	break;
		// }
		// if (scan < 10) // Breaks by distance
		// {
		// 	bluetooth.println("Obsticle: Right");
		// 	break;
		// }

		// new code
		this->moveServo(45);
		scan = this->pulse();
		this->moveServo(90);
		scanl = this->pulse();
		this->moveServo(135);
		scanr = this->pulse();
		bluetooth.print("Forward Scan distance: ");
		bluetooth.println(scan);
		bluetooth.print("Left Scan distance: ");
		bluetooth.println(scanl);
		bluetooth.print("Right Scan distance: ");
		bluetooth.println(scanr);
		if (scan < 8) // Breaks by distance
		{
			bluetooth.println("Obsticle: Forward");
			break;
		}
		else if (scanl < 3)
		{
			bluetooth.println("Obsticle: Left");
			break;
		}
		else if (scanr < 3)
		{
			bluetooth.println("Obsticle: Right");
			break;
		}
		else if (bluetooth.available() > 0) // Breaks by input
		{
			bluetooth.println("Break Recieved");
			break;
		}
	}
	// emptys bluetooth
	while (bluetooth.available() > 0)
	{
		bluetooth.read();
	}
	this->stop(); // Stops robot
}

void Robot::moveServo(byte degree)
/*

	Robot::moveServo

	Moves servo to selected angle and checks for succsessful rotation

	@param: byte degree ~ the degree the servo will go to

*/
{
	while (this->servo.read() != degree)
	{
		this->servo.write(degree);
	}
}

unsigned int Robot::pulse()
/*
	Robot::pulse

	takes a quick pulse and returns distance

	returns: unsinged int distance
*/
{
	int pulse;
	int pulse2;
	int pulseAvg;
	digitalWrite(this->trig, LOW);
	digitalWrite(this->trig, HIGH);
	pause(1);
	digitalWrite(this->trig, LOW);
	pulse = pulseIn(this->echo, HIGH);
	digitalWrite(this->trig, LOW);
	digitalWrite(this->trig, HIGH);
	pause(1);
	digitalWrite(this->trig, LOW);
	pulse2 = pulseIn(this->echo, HIGH);
	pulse = pulse / 29 / 2;
	pulse2 = pulse2 / 29 / 2;
	pulseAvg = (pulse + pulse2) / 2;
	if (abs(pulse - pulse2) >= 40)
	{
		bluetooth.println("Recurring") unsigned int recursion = this->pulse();
		return recursion;
	}
	return pulse;
}

void Robot::scan()
/*
	Robot::scan

	This function scans the whole 180 degrees with the pulse function
	Less accurate but faster version of scanLongPulse
	it then updates the distances variables

*/
{
	unsigned int distance;
	this->moveServo(0);
	pause(1000);
	for (byte i = 0; i < 179; ++i)
	{
		this->moveServo(i);
		distance = this->pulse();
		this->distances[i] = distance;
		// if (distance < 3){
		// 	return i;
		// }
	}
}

int Robot::checkBluetooth()
/*
	Robot::checkBluetooth

	takes input from serial, responds and returns

	returns: input from serial/Bluetooth
*/
{
	int input;
	bluetooth.println("Looking for input!");
	while (true)
	{
		if (bluetooth.available() > 0)
		{
			input = bluetooth.read();
			bluetooth.println("Input Recieved: ");
			bluetooth.println(input);
			return input;
		}
		else if (Serial.available() > 0)
		{
			input = Serial.read();
			Serial.println("Input Recieved: ");
			Serial.println(input);
			input -= 48; // required for ascii to int conversion
			bluetooth.println("Input processed: ");
			bluetooth.println(input);
			return input;
		}
	}
}

void Robot::remoteControl()
/*
	Robot::remoteControl

	remote control method

	this takes an input and excecutes then askes for a new input

	Possible inputs:
		-> -1 ~ exit
		-> 0  ~ move Forward
		-> 1  ~ move Back
		-> 2  ~ turn Left
		-> 3  ~ turn Right
		-> 4  ~ enter pathfinding mode
		-> 10 ~ pass
*/
{
	byte state = 9;
	while (true)
	{
		state = this->checkBluetooth();
		switch (state)
		{
		case 0:
			bluetooth.println("Robot: Moving Forward");
			this->forward();
			this->moveServo(90);
			break;
		case 1:
			bluetooth.println("Robot: Moving Back");
			this->back();
			break;
		case 2:
			bluetooth.println("Robot: Moving Left");
			this->left();
			break;
		case 3:
			bluetooth.println("Robot: Moving Right");
			this->right();
			break;
		case 4:
			bluetooth.println("Robot: Entering Pathfinding");
			this->pathfinding();
			break;
		case 5:
			bluetooth.println("Robot: Scanning");
			this->stop();
			this->scan();
			this->printDistances();
			this->moveServo(90);
			break;
		case 6:
			bluetooth.println("Robot: Stopping");
			this->moveServo(90);
			this->stop();
		}
	}
}

void Robot::pathfinding()
/*
	Robot::pathfinding

	This function finds its way arround a maze autonomously
*/
{
}

void Robot::printDistances()
{
	for (byte i = 0; i < 179; ++i)
	{
		bluetooth.print("Distance as degree: ");
		Serial.print("Distance as degree: ");
		bluetooth.print(i);
		Serial.print(i);
		bluetooth.print(" is ");
		Serial.print(" is ");
		bluetooth.println(this->distances[i]);
		Serial.println(this->distances[i]);
	}
}

// Creates instance of Robot named mike
Robot mike;

void setup()
{
	// sets up serial and bluetooth
	Serial.begin(9600);
	bluetooth.begin(9600);
	bluetooth.println("Hi");
	Serial.println("init testing"); 

	// mike stuff
	mike.init(definedMotors, definedMisc); // initializes
	mike.remoteControl(); // remote control
}

void loop()
{
	break; // breaks the program
}
