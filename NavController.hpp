/*
 * NavController.hpp
 *
 *  Created on: Feb 1, 2018
 *      Author: Cloie
 * Power Up 2018
 * Nav Contoller hpp holds the header files from cpp
 * Change Log: August 28, 2018- added descriptions and comments.
 */

#ifndef SRC_NAVCONTROLLER_HPP_
#define SRC_NAVCONTROLLER_HPP_

#include "Pose2D.hpp"
#include "Vector2D.hpp"
#include "VelocityVector.hpp"

class PIDOutputCatcher: public PIDOutput
{
public:
	/*catches the ouputs from PID Controller*/
	PIDOutputCatcher()
	{
		_output=0.0;
	}
	virtual ~PIDOutputCatcher()
	{

	}

	void PIDWrite(double output)
	{
		std::lock_guard<wpi::mutex> sync(m_mutex);
		_output = output;
	}

	/*Gets output from PID controller*/
	double GetOutput()
	{
		std::lock_guard<wpi::mutex> sync(m_mutex);
		return _output;
	}
private:
	double _output;
	mutable wpi::mutex m_mutex;
	//LinearDigitalFilter filt{};
};

class NavController
{
	Pose2D* robotPose;
	double v0;
	double alpha;
	double maxOmega;

	double _angle;
	PIDController headingPID;
	PIDOutputCatcher catcher;


public:
	NavController(Pose2D* pose);
	NavController(Pose2D* pose,double p, double i, double d);

	/*sets PID, V0, Alpha, Omega, and Angle, also updates angle*/
	void SetPID(double p, double i, double d);
	void SetPID(double f,double p, double i, double d);
	void SetV0(double val){v0=val;};
	void SetAlpha(double val){alpha=val;};
	void SetOmega(double val){maxOmega=val; headingPID.SetOutputRange(-val,val);};
	void SetAngle(double angle);
	void UpdateAngle(double angle);

	/*Sets and gets PID*/
	void 	SetP(double p){headingPID.SetP(p);};
	void 	SetI(double i){headingPID.SetI(i);};
	void 	SetD(double d){headingPID.SetD(d);};
	double 	GetP(){return headingPID.GetP();};
	double 	GetI(){return headingPID.GetI();};
	double 	GetD(){return headingPID.GetD();};

/* gets V0, Alpha, Omega, Angle, and PID Error*/
	double GetV0(){return v0;};
	double GetAlpha(){return alpha;};
	double GetOmega(){return maxOmega;};
	double GetAngle(){return _angle;};\
	double GetPIDError(){return headingPID.GetError();};

/*TURN*/
	double Turn(double phi);
	double Turn();
	Vector2D GoToGoal(double x, double y);

};





#endif /* SRC_NAVCONTROLLER_HPP_ */
