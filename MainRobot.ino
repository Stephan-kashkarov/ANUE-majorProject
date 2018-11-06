/* ANU MAZERUNNER ROBOT CODE
	this code was written for ANUE
	by: Stephan Kashkarov 
	in: October 2018
*/
#include <Servo.h>

class Robot
/*
	Class Robot

	This class contains all of the functions for the robot
	Its variables are:

		-> servo     | An instance of the servo class
		-> trig      | The trig pin of the ultrasonic sensor
		-> echo      | The echo pin of the ultrasonic sensor
		-> motorPins | An array contaning all the motor motorPins
					 | Ordered: Top-left Bottom-left, Top-right, Bottom-right
		-> distances | An array cantaining 180 different distance points
	*/
{
	private:
		Servo servo;
		byte trig;
		byte echo;
		byte motorPins[4];
		int distances [180];

	public:
		void init(byte motors[4], byte misc[3])
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
			this->trig         = misc[1];
			this->echo         = misc[2];
			this->servo.attach(misc[0]); // attaches servo to servo pin

			for(byte i = 0; i < 4; ++i)
			{
				pinMode(this->motorPins[i], OUTPUT); // sets all pins to outputs
			}
			
			// more pinmodes
			pinMode(misc[0], OUTPUT);
			pinMode(this->trig,  OUTPUT);
			pinMode(this->echo,  INPUT);
			
		}

		void left(int t)
		/*
			Robot::left

			This function turns left for T time

			@param int t ~ The time that the operation will go for in milliseconds
		*/
		{
			digitalWrite(this->motorPins[0], LOW);
			digitalWrite(this->motorPins[1], HIGH);
			digitalWrite(this->motorPins[2], HIGH);
			digitalWrite(this->motorPins[3], LOW);
			delay(t);
		}

		void right(int t)
		/*
			Robot::right

			This function turns right for T time

			@param int t ~ The time that the operation will go for in milliseconds
		*/
		{

			digitalWrite(this->motorPins[0], HIGH);
			digitalWrite(this->motorPins[1], LOW);
			digitalWrite(this->motorPins[2], LOW);
			digitalWrite(this->motorPins[3], HIGH);
			delay(t);
				
		}

		void forward(int t)
		/*
			Robot::forward

			This function turns forward for T time while sensing

			@param int t ~ The time that the operation will go for in milliseconds
		*/
		{
			unsigned long timer = millis();
			while ((timer+t) > millis())
			{
				if(this->quickPulse(90) < 3)
				{
					Serial.println("Movement Obstructed");
					break;
				}
				digitalWrite(this->motorPins[0], HIGH);
				digitalWrite(this->motorPins[1], LOW);
				digitalWrite(this->motorPins[2], HIGH);
				digitalWrite(this->motorPins[3], LOW);
			}
		}

		void back(int t)
		/*
			Robot::back

			This function turns back for T time while sensing

			@param int t  The time that the operation will go for in milliseconds
		*/
		{
			unsigned long timer = millis();
			while ((timer+t) > millis())
			{
				if(this->quickPulse(90) < 3){
					Serial.println("Movement Obstructed");
					break;
				}
				digitalWrite(this->motorPins[0], LOW);
				digitalWrite(this->motorPins[1], HIGH);
				digitalWrite(this->motorPins[2], LOW);
				digitalWrite(this->motorPins[3], HIGH);
			}
			
		}

		unsigned int quickPulse(byte degree)
		/*
			Robot::quickPulse

			takes a quick pulse at the selected angle and returns distance

			@param: byte degree ~ the degree the servo will go to

			returns: unsinged int distance
		*/
		{
			while (this->servo.read() != degree){
				this->servo.write(degree);
			}
			digitalWrite(this->trig, LOW);
			digitalWrite(this->trig, HIGH);
			delay(1);
			digitalWrite(this->trig, LOW);
			int pulse = pulseIn(this->echo, HIGH);
			pulse = pulse / 29 / 2;
			this->distances[degree] = pulse;
			return pulse;
		}

		unsigned int longPulse(byte degree)
		/*
			Robot::longPulse

			takes 5 quick pulse at the selected angle and returns average distance
			this method is more accutate and will remove issues with random interference

			@param: byte degree ~ the degree the servo will go to

			returns: unsinged int average distance
		*/
		{
			int pulseAvg;
			while (this->servo.read() != degree)
			{
				this->servo.write(degree);
			}

			for (byte i = 0; i < 3; ++i)
			{
				digitalWrite(this->trig, HIGH);
				delay(1);
				digitalWrite(this->trig, LOW);
				int pulse = pulseIn(this->echo, HIGH);
				pulse = pulse / 29 / 2;
				pulseAvg += pulse;
				pulse = 0;
			}
			pulseAvg = pulseAvg / 3;
			this->distances[degree] = pulseAvg;
			return pulseAvg;
		}

		unsigned int scanQuickPulse()
		/*
			Robot::scanQuickPulse

			This function scans the whole 180 degrees with the quickPulse function
			Less accurate but faster version of scanLongPulse
			it then updates the distances variables

			returns:
			if a distance is lower the  cutoff then the degree is returned
		*/
		{
			unsigned int distance;
			while (this->servo.read() != 0)
			{
				this->servo.write(0);
			}
			delay(100);
			for (byte i = 0; i < 180; ++i)
			{
				delay(10);
				distance = this->quickPulse(i);
				this->distances[i] = distance;
				if (distance < 3){
					return i;
				}
			}
		}

		unsigned int scanLongPulse()
		/*
			Robot::scanQuickPulse

			This function scans the whole 180 degrees with the longPulse function
			it then updates the distances variables

			returns:
			if a distance is lower the  cutoff then the degree is returned
		*/
		{
			unsigned int distance;
			while (this->servo.read() != 0)
			{
				this->servo.write(0);
			}
			delay(100);
			for (byte i = 0; i < 180; ++i)
			{
				delay(10);
				distance = this->longPulse(i);
				this->distances[i] = distance;
				if (distance < 3){
					return i;
				}
				
			}
		}

		int checkBluetooth()
		/*
			Robot::checkBluetooth

			takes input from serial, responds and returns

			returns: input from serial/Bluetooth
		*/
		{
			int input;
			while (true)
			{
				if (Serial.available() > 0)
				{
					input = Serial.read();
					Serial.println("Input Recieved: ");
					Serial.println(input);
					return input;
				}
			}
		}

		void remoteControl()
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
				-> 5  ~ enable debug mode
				-> 10 ~ pass
		*/
		{
			int state = 10;
			while (state != -1)
			{
				state = this->checkBluetooth();
				switch (state)
					{
						case 0:
							this->forward(100);
							state = 10;
							break;
						case 1:
							this->back(100);
							state = 10;
							break;
						case 2:
							this->left(100);
							state = 10;
							break;
						case 3:
							this->right(100);
							state = 10;
							break;
						case 4:
							this->pathfinding();
							state = -1;
							break;
						case 5:
							this->debug();
							state = -1;
							break;

						default:
							break;
					}

			}
		}

		void pathfinding()
		/*
			Robot::pathfinding

			This function finds its way arround a maze autonomously
		*/
		{

		}
		void debug()
		/*
			Robot::debug

			This method enables the debug menu through bluetooth
			
			Enabled options:
				-> print distances
				-> TODO
		*/
		{

		}
};


byte definedMotors[4] = {8, 9, 10, 11};
byte definedMisc[3] = {3, 4, 5};

// Creates instance of Robot named mike
Robot mike;


void setup()
{
	mike->init(definedMotors, definedMisc);
	mike->remoteControl();
}

void loop()
{
	mike->pathfinding();
}
