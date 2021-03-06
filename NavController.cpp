
/*
 * NavController.cpp
 *
 *  Created on: Feb 3, 2018
 *      Author: Cloie
 * Power Up 2018
 * Nav Controller takes the position of the robot and a
 * targeted position and calculates how to get there which
 * is called the error.
 * Change Log: August 28, 2018- Added descriptions and comments
 */
#include "NavController.hpp"
#include <cmath>

/*Sets v0, alpha, robotPose, maxOmega, and _angle*/
NavController::NavController(Pose2D* pose):
	headingPID(2000.0,0.0,10000.0,pose,&catcher,.005),
	catcher()
{

	v0 = 12.0;
	alpha = 1.0;
	robotPose = pose;
	maxOmega = 2000.0;
	_angle  = 0.0;

	/*Puts the numbers into the PID controller.*/
	headingPID.SetContinuous();
	headingPID.SetInputRange(-3.14159,3.14159);
	headingPID.SetOutputRange(-maxOmega,maxOmega);

	headingPID.SetPercentTolerance(.02);

	headingPID.Enable();

}
NavController::NavController(Pose2D* pose,double p, double i, double d):
	headingPID(p,i,d,robotPose,&catcher,.005),
	catcher()
{
	/*updates info into PID controller.*/
	v0 = 12.0;
	alpha = 1.0;
	robotPose = pose;
	maxOmega = 400.0;
	_angle  = 0.0;

	headingPID.SetContinuous();
	headingPID.SetInputRange(-3.14159,3.14159);
	headingPID.SetOutputRange(-maxOmega,maxOmega);
	headingPID.SetPercentTolerance(.02);
	headingPID.Enable();
}

void  	NavController::SetPID(double p, double i, double d)
{
	/*Sets PID*/
	headingPID.SetP(p);
	headingPID.SetI(i);
	headingPID.SetD(d);
}
void  	NavController::SetPID(double f,double p, double i, double d)
{
	headingPID.SetF(f);
	headingPID.SetP(p);
	headingPID.SetI(i);
	headingPID.SetD(d);
}
/*sets angle and puts it into the PID*/
void 	NavController::SetAngle(double angle)
{
	_angle=atan2(sin(angle),cos(angle));
	headingPID.SetSetpoint(_angle);
	headingPID.Reset();
	headingPID.Enable();
}

/*Updates angle into PID*/
void 	NavController::UpdateAngle(double angle)
{
	_angle=atan2(sin(angle),cos(angle));
	headingPID.SetSetpoint(_angle);
}

/*GO TO GOAL*/
Vector2D 	NavController::GoToGoal(double x, double y)
{
	/*Error of x minus the x robot position, error of y minus y robot position.*/
	double ex = x-robotPose->GetX();
	double ey = y-robotPose->GetY();
	double emag = sqrt(ex*ex+ey*ey);
	/*Math equation:*/
	double k=v0*(1.0-exp(-alpha*emag*emag));
	double w = atan2(ey,ex)-robotPose->GetPhi();
	double dir = 1.0;

	/*if the error of x is less than 0.0 and the absolute values of w is greater
	 * than pi divided by two then change the direction to negative 1.0*/

	if(ex<0.0 && (fabs(w)>3.14159/2.0))
	{
		dir = -1.0;
	}
	w = Turn();

	/*if the error of x is less than 12.00 then omega-turn- equals 0.0*/
	if(ex<12.0)
	{
		w=0.0;
	}
	//return Vector2D(k*ex,k*ey);
	//return Vector2D(dir*v0*(1.0-exp(-alpha*e*e)),1.0*w);
	return Vector2D(dir*k,w);

}

double 	NavController::Turn(double phi)
{
	double e=phi - robotPose->GetPhi();

	e=atan2(sin(e),cos(e));

	if(e>2.0)
	{
		e = 3.14159;
	}
	if(e<-2.0)
	{
		e = -3.14159;
	}
	return e;
}
double 	NavController::Turn()
{
	double out = catcher.GetOutput();
	out = atan2(sin(headingPID.GetError()),cos(headingPID.GetError()));//get error angle
	out = maxOmega*out;
	SmartDashboard::PutNumber("Turn PID output", out);//puts PID output into smart dashboard
	SmartDashboard::PutNumber("fabs err", fabs(headingPID.GetError()));//Puts error in sd
	if(fabs(headingPID.GetError())>.02 && fabs(out)<550.0)
	{
		SmartDashboard::PutString("Debug Statement", "YUP!");

		if(out<0)
		{
			out = -650.0;
		}else
		{
			out = 650.0;
		}

	}else
	{
		SmartDashboard::PutString("Debug Statement", "NOPE!");
	}
	return out;
}
