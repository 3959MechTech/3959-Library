/*
 * UnicycleController.hpp
 *
 *  Created on: Jan 27, 2018
 *      Author: Austin
 */
/*Author: Austin
 *
 *Date: 2018 Power Up
 *
 *Description: This code helps to navigate the field in auto. Through out this code many things
 *used or created. One of two things are the variables x and y. Bothof these variables work together
 *to determine position. The second thing is Phi. Phi is a heading used as a sense of direction for
 *movement along the cartesian graph which is created by the variables x and y.
 *
 *Change Log: August 28, Code was commented and change log along with description were added: Austin
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


	UnicycleController(Pose2D* pose);// the position of the robot on the field during auto.

// Description: DifferetialMotorCommand uses the cordinates x and y and uses phi as a heading

	DifferentialMotorCommand GetDifferentialMotorCommand(double x,double y);
	DifferentialMotorCommand GetDifferentialMotorCommand(double phi);

// Description: Omega is the velocity and speed of a turn


	void Set_maxOmega(double w){_maxOmega = w;};
	double Get_maxOmega(){return _maxOmega;};

/* Description: The wheel base is the distance from the center of the robots
wheel to the direct center of the robot*/


	void Set_wheelBase(double l){_wheelBase =l;};
	double Get_wheelBase(){return _wheelBase;};

// Description: Wheel Radius is the distance in inches of diameter of the wheel


	void Set_wheelRadius(double r){_wheelRadius =r;};
	double Get_wheelRadius(){return _wheelRadius;};

/* Description: Transform L is also the distance from the center of the wheel to the very center of the robot.*/

	void Set_transformL(double l){_transformL =l;};
	double Get_transformL(){return _transformL;};

private:

	Pose2D* robotPose;// position on the field
	double _maxOmega; //this is used to scale turn speed

	double _wheelBase;// R=WheelBase
	double _wheelRadius;//L=WheelRadius

	double _transformL;// WheelBase

public:
	VelocityVector Tracker(double x,double y);
	VelocityVector Tracker(double phi);
	VelocityVector Transform (double x,double y );
	VelocityVector Transform(double phi );

	DifferentialMotorCommand DifferentialOutput(VelocityVector input);

};


#endif //SRC_UNICYCLECONTROLLER_HPP_




