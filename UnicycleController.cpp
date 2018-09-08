/*
 * uni.cpp
 *
 *  Created on: Jan 27, 2018
 *      Author: Austin
 */


/* Author: Austin
 *
 *Date: 2018 Power Up
 *
 *Description:This code helps to navigate the field in auto.
 *
 *Summary:Through out this code many things used or created.One of two things are the variables
 *x and y. Both of these variables work together to determine position.The second thing is Phi.
 *Phi is a heading used as a sense of direction for movement along the cartesian graph which is
 * created by the variables x and y.
 *
 *Change Log: August 28, Code was commented and change log along with description were added
 */


#include <DifferentialMotorCommand.hpp>// able to call functions from these four files //
#include "UnicycleController.hpp"
#include "VelocityVector.hpp"
#include <math.h>

//Description: These are variables that determine the robots position on the field

UnicycleController::UnicycleController(Pose2D* pose)//constructor
{
	robotPose = pose;
	_maxOmega = 1500.0;
	_wheelBase=24.0;
	_wheelRadius=2.0;
	_transformL = .1;
}

//Description: This part of the code keeps track of the robots pose on the field using coordinates


 VelocityVector UnicycleController::Tracker(double x,double y)
 {
 	VelocityVector output;
 	output.v = sqrt(x*x+y*y);
 	output.w = _maxOmega*atan2(y,x);


 	return output;
 };

//Description: This part of the code keeps track of the robots pose on the field using heading


 VelocityVector UnicycleController::Tracker(double phi)
 {
	VelocityVector output;
	output.v=0.0;
	phi = atan2(sin(phi),cos(phi));// normalize phi between PI and -PI //

	output.w = phi*_maxOmega;

	return output;
 }

//Description:  thsi code uses theta ( a vairable ) along with x and y  to get the output.v equation


 VelocityVector UnicycleController::Transform(double x,double y)

 {
 	VelocityVector output;

 	double theta = atan(y/x);

 	double l = _transformL;

 	output.v = x*cos(theta)+y*sin(theta);
 	output.w = y*cos(theta)/l-x*sin(theta)/l;

 	return output;
 }

//Description: sets the ouput.w to -phi

 VelocityVector UnicycleController::Transform(double phi )
 {
 	VelocityVector output;
 	output.v=0.0;
 	phi = atan2(sin(phi),cos(phi));

 	output.w = -phi*_maxOmega;

 	return output;
 }

//Description:commanding the right (VR) and left(VL) motors using input.v and input. w


 DifferentialMotorCommand UnicycleController::DifferentialOutput(VelocityVector input)
{

	DifferentialMotorCommand command;
	command.VR=(2*input.v+input.w)/2;
    command.VL=(2*input.v-input.w)/2;
	return command;
}

//Description: Returns the equation from differential output of the tracker(X,Y)

 DifferentialMotorCommand UnicycleController::GetDifferentialMotorCommand(double x,double y)
 {

	return DifferentialOutput(Tracker(x,y));

 }

//Description: p = atan using sin and cos of phi for the tracker

 DifferentialMotorCommand UnicycleController::GetDifferentialMotorCommand(double phi)
 {
	 double p = atan2(sin(phi), cos(phi));
	 return DifferentialOutput(Tracker(p));
 }

