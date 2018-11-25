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
byte definedMisc[3] = {3, 4, 5};        // servo, trig, echo
byte commPins[2] = {12, 13};            // bluetooth



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
	// mike.init(definedMotors, definedMisc); // initializes
	// mike.remoteControl();				   // remote control
}

void loop()
{
}
