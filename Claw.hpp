/*
 * Claw.hpp
 *
 *  Author: Brandon Fleming / Reuben
 *
 *  Date: Power Up 2018
 *
 *  Description: The claw hpp file for all the commands being used by the claw made for a rookie to understand
 *
 *  Summary: The claw is using motors to shoot the cube and feed it in, it uses a sensor to detect if it HAS the cube.
 *  It has things like a timer, a firing bool and feed/shoot command to create a functioning claw.
 *
 *  Change Log 8/27/18: Fully commented and add descriptions to be understood easier
 *
 */

#ifndef CLAW_HPP_
#define CLAW_HPP_

#include <WPILib.h> // gets the main header file
#include <ctre/phoenix.h> // gets the phoenix header


class Claw // the claw class
{

private:
	TalonSRX lm, rm; // the left and right motot
	DigitalInput sensor; // the sensor

	Timer 	fireTimer; // the fire timer for firing
	double 	fireTime; // float but 2x so instead of 4 bits its 8bits, the fire time to shoot

public:
	Claw(int leftMotor, int rightMotor, int sensor); // the id of the motor and sonar
    void Shoot(double speed); // shoots the cube but with the speed with the motors
    bool Feed(double speed); // same as shooting but inverted

    void ResetFire(); // resets the fire timer
    bool isFiring(); // is the cube being fired
    void Fire(double speed, double dur); // the speed of the fire and dur


    void SendData(std::string name = "Claw"); // send back the data
};
#endif /* CLAW_HPP_ */
