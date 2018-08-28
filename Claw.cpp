/*
 * Claw.cpp
 *
 *  Author: Brandon Fleming
 *
 *  Date: Power Up 2018
 *
 *  Description: The claw cpp file for all the commands being used by the claw made for a rookie to understand
 *
 *  Summary: The claw is using motors to shoot the cube and feed it in, it uses a sensor to detect if it HAS the cube.
 *  It has things like a timer, a firing bool and feed/shoot command to create a functioning claw.
 *
 *  Change Log 8/27/18: Fully commented and add descriptions to be understood easier
 *
 */

#include "Claw.hpp" // includes the claw.hpp file

Claw::Claw(int leftMotor, int rightMotor, int sensorport): lm(leftMotor), rm(rightMotor), sensor(sensorport) // constructor for the main claw
{
	lm.SetInverted(false); // sets the invert left motor as false
	//rm.Follow(lm);
	rm.SetInverted(true); // sets the invert right motor as true

	fireTime = 0.5; // fire time
}

void Claw::Shoot(double speed) // shoot function
{
	lm.Set(ControlMode::PercentOutput, speed); // the force being shot with by the left motor
	rm.Set(ControlMode::PercentOutput, speed); // the force being shot with by the right motor
}

bool Claw::Feed(double speed) // fire feed time
{
/*
	if(!sensor.Get())
	{
*/		lm.Set(ControlMode::PercentOutput, -speed*.8); // the force being used to feed in the cube by the left motor
		rm.Set(ControlMode::PercentOutput, -speed); // the force being used to feed in the cube by the right motor
		return true; // returns as true
/*	}else
	{

		lm.Set(ControlMode::PercentOutput, -0.2);
		rm.Set(ControlMode::PercentOutput, -0.3);
		return false;
	}
*/
}

void Claw::ResetFire() // resets the fire timer to shoot
{
	fireTimer.Stop(); // stops the fire timer
}

void Claw::Fire(double speed, double dur) // fire function
{
	fireTime = dur;
	fireTimer.Start(); // starts the fire timer
	Shoot(speed); // shoots the cube with the speed float

}

bool Claw::isFiring() // the isfiring bool detects whether it is being shot
{
	if(fireTimer.Get()>=fireTime) // uses math to detect whether the fire timer is greater than fire time
	{
		Shoot(0.0); // shoots the cube
		fireTimer.Stop(); // stops the fire timer
		return false;
	}
	return true; // returns as  true
}

void Claw::SendData(std::string name) // sends the data using the std namespace
{
	SmartDashboard::PutBoolean(name + " Sensor",sensor.Get()); // gets the sensor name on the smart dashboard
	//SmartDashboard::PutNumber(name + " Speed",lm.);
}
