/*
 * Richard J. Bradshaw IV
 *
 * Date:8/28/2018
 *
 * Description: This hpp is a constructor for most of the functions we are going to use for the
 * Elevator cpp
 *
 * Change log:
 * Date:8/28/2018 Commented all the code
 */

#ifndef SRC_ELEVATOR_HPP_
#define SRC_ELEVATOR_HPP_

#include <ctre/phoenix.h>
#include <WPILib.h>
#include <string>

class Elevator
{
public:

	enum EPos
	{
		Bottom = 0,
		Travel = 1,
		StackedBlock = 2,
		Switch = 3,
		ScaleLow = 4,
		ScaleMedium = 5,
		ScaleHigh = 6,
		Top = 7,
	};

private:
	static const int MaxPos = 8;
	static const int kTimeOut = 10;

	TalonSRX 	eTalon,
				eSTalon;

	double posVals[MaxPos][3];//0 = position, 1 = ramp, 2 = max speed


	//bool eZeroed;

	double maxRamp;

	EPos current;



public:


	Elevator(int eTalon, int eSTalon);//a constructor for the talons

	void SendData(std::string name="Elevator");

	double GetHeight(EPos);//gets height of Elevator based on encoder ticks
	double GetRamp(EPos);//gets ramp
	double GetMaxSpeed(EPos);//Gets the speed of the elevator

	bool GetBottomLimitSwitch(){return eTalon.GetSensorCollection().IsRevLimitSwitchClosed();};
	bool GetBottomSlaveLimitSwitch(){return !eSTalon.GetSensorCollection().IsRevLimitSwitchClosed();};
	bool GetTopLimitSwitch(){return eTalon.GetSensorCollection().IsFwdLimitSwitchClosed();};
	bool GetTopSlaveLimitSwitch(){return eSTalon.GetSensorCollection().IsFwdLimitSwitchClosed();};

	void SetMaxRamp(double ramp);//allows you to set the ramp with any number from -1 to 1

	//void eZeroed();

	EPos GetEPos();//gets elevator position
	double GetError();//gets difference from where we are and where we are supposed to be

	double GetEncoderPos();//gets raw encoder positions
	double GetSetPoint();

	void SetEPos(EPos);//lets you set the elevator positions

	void incPos();//increment Elevator positions

	void decPos();//decrement elevator positions

	void SetF(double f, int slot){eTalon.Config_kP(slot,f,kTimeOut);};
	void SetP(double p, int slot){eTalon.Config_kP(slot,p,kTimeOut);};
	void SetI(double i, int slot){eTalon.Config_kI(slot,i,kTimeOut);};
	void SetD(double d, int slot){eTalon.Config_kD(slot,d,kTimeOut);};

	void SetMotorSpeed(double speed);//lets you set motor speed
	void SetPosition(double pos);//gives direct control of PID Controller

};


#endif /* SRC_ELEVATOR_HPP_ */
