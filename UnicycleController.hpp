/*
 * UnicycleController.hpp
 *
 *  Created on: Jan 27, 2018
 *      Author: Austin
 */
/* )Author: Austin
   )Date: 2018 Power Up
   )Description:This code helps to navigate the field in auto.
   )Summary:Through out this code many things used or created. One of two things are the variables x and y. Both
   of these variables work together to determine position. The second thing is Phi. Phi is a heading used as a sense
   of direction for movement along the cartesian graph which is created by the variables x and y.
   )Change Log: August 28, Code was commented and change log along with description were added
 */






#ifndef SRC_UNICYCLECONTROLLER_HPP_
#define SRC_UNICYCLECONTROLLER_HPP_

#include"DifferentialMotorCommand.hpp"// all #includes are other documents created by fellow teammates or c++ devs
#include"VelocityVector.hpp"
#include"Pose2D.hpp"
#include <math.h>

#include "Pose2D.hpp"
//class Pose2D{};


class UnicycleController// initialization of this class
{

public:


	UnicycleController(Pose2D* pose);



	DifferentialMotorCommand GetDifferentialMotorCommand(double x,double y);/* this is getting the ability
	to use the motors using the x and y variable*/
	DifferentialMotorCommand GetDifferentialMotorCommand(double phi);/* this time the phi variable is in place of x
	and y */

	void Set_maxOmega(double w){_maxOmega = w;}; // max turn speed
	double Get_maxOmega(){return _maxOmega;}; // return max turn speed

	void Set_wheelBase(double l){_wheelBase =l;}; // distance" from the center of the wheel to the robot center
	double Get_wheelBase(){return _wheelBase;}; // return the distance in inches

	void Set_wheelRadius(double r){_wheelRadius =r;};// the radius of the robots wheel measured in inches
	double Get_wheelRadius(){return _wheelRadius;}; // return the radius of the wheel in inches

	void Set_transformL(double l){_transformL =l;}; //wheel base
	double Get_transformL(){return _transformL;}; // return wheel base

private:

	Pose2D* robotPose;
	double _maxOmega; //this is used to scale turn speed

	double _wheelBase;// R=WheelBase
	double _wheelRadius;//L=WheelRadius

	double _transformL;//l

public:
	VelocityVector Tracker(double x,double y);
	VelocityVector Tracker(double phi);
	VelocityVector Transform (double x,double y );
	VelocityVector Transform(double phi );

	DifferentialMotorCommand DifferentialOutput(VelocityVector input);

};


#endif //SRC_UNICYCLECONTROLLER_HPP_




