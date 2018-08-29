/*
 * Elevator.cpp
 * Author: Richard Bradshaw
 * Created on: Feb 8, 2018
 * Description: This class controls the elevator.
 * Change log: 
 * Date: 8/28/2018 commented all the code
 */
#include "Elevator.hpp"

#include <ctre/phoenix.h>
#include <WPILib.h>
#include <string>

Elevator::Elevator(int masterMotor, int slaveMotor):eTalon(masterMotor),eSTalon(slaveMotor)
{
	//A parameter value of "1" will enable the feature while a parameter of "0" will disable the feature.
	//If limit switch is pressed the motor won't go past
	eTalon.ConfigSetParameter(ctre::phoenix::ParamEnum::eClearPositionOnLimitR,0,0,0,kTimeOut);
	eSTalon.ConfigSetParameter(ctre::phoenix::ParamEnum::eClearPositionOnLimitR,0,0,0,kTimeOut);

	eTalon.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, kTimeOut);
/*
	eTalon.ConfigForwardLimitSwitchSource(
			LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
			LimitSwitchNormal::LimitSwitchNormal_NormallyClosed,
			//eSTalon.GetDeviceID(),
			10
		);
	eTalon.ConfigReverseLimitSwitchSource(
			LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
			LimitSwitchNormal::LimitSwitchNormal_NormallyClosed,
			//eSTalon.GetDeviceID(),
			10
		);
*/
	eSTalon.Follow(eTalon);

	//Elevator Inversion
	eTalon.SetInverted(false);
	eSTalon.SetInverted(false);

	eTalon.SetSensorPhase(false);

	maxRamp=1.0;
	current = EPos::Bottom;

	posVals[Bottom][0] = 1000;//the encoder Position
	posVals[Bottom][1] = .2;//Ramp speed for robot drive
	posVals[Bottom][2] = 1.0;//max speed for robot

	posVals[Travel][0] = 8000;//the encoder Position
	posVals[Travel][1] = .2;//Ramp speed for robot drive
	posVals[Travel][2] = 1.0;//max speed for robot

	posVals[StackedBlock][0] = 15000;//the encoder Position
	posVals[StackedBlock][1] = .2;//Ramp speed for robot drive
	posVals[StackedBlock][2] = 1.0;//max speed for robot

	posVals[Switch][0] = 45000;//the encoder Position
	posVals[Switch][1] = .3;//Ramp speed for robot drive
	posVals[Switch][2] = 1.0;//max speed for robot

	posVals[ScaleLow][0] = 120000;//the encoder Position
	posVals[ScaleLow][1] = .5;//Ramp speed for robot drive
	posVals[ScaleLow][2] = .7;//max speed for robot

	posVals[ScaleMedium][0] = 137000;//the encoder Position
	posVals[ScaleMedium][1] = .6;//Ramp speed for robot drive
	posVals[ScaleMedium][2] = .6;//max speed for robot

	posVals[ScaleHigh][0] = 156000;//the encoder Position
	posVals[ScaleHigh][1] = .75;//Ramp speed for robot drive
	posVals[ScaleHigh][2] = .4;//max speed for robot

	posVals[Top][0] = 156000;//the encoder Position
	posVals[Top][1] = 1.0;//Ramp speed for robot drive
	posVals[Top][2] = .4;//max speed for robot

	eTalon.ConfigPeakOutputForward(1.0,0);//max speed going up
	eTalon.ConfigPeakOutputReverse(-.7,0);//max speed going down
	eTalon.ConfigNominalOutputForward(.06,0);
	eTalon.ConfigNominalOutputReverse(0.06,0);//I think this will help with the lowering.

	eTalon.SelectProfileSlot(0,0);

	//Up moves
	eTalon.Config_kF(0,0,kTimeOut);
	eTalon.Config_kP(0,.6,kTimeOut);//original @.15
	eTalon.Config_kI(0,0,kTimeOut);
	eTalon.Config_kD(0,0.2,kTimeOut);//original @ .2
	//uses slots to configure the FPID
	//Down Moves
	eTalon.Config_kF(1,0,kTimeOut);
	eTalon.Config_kP(1,.3,kTimeOut);//original @.05
	eTalon.Config_kI(1,0,kTimeOut);
	eTalon.Config_kD(1,0.8,kTimeOut);

}

void Elevator::SendData(std::string name)
{

	SmartDashboard::PutNumber(name+" Velocity", eTalon.GetSelectedSensorVelocity(0));
	SmartDashboard::PutNumber(name+" Position", eTalon.GetSelectedSensorPosition(0));
	SmartDashboard::PutNumber(name+" Slave Position", eSTalon.GetSelectedSensorPosition(0));

	SmartDashboard::PutNumber(name+" raw position", eTalon.GetSensorCollection().GetQuadraturePosition());
	SmartDashboard::PutBoolean(name+" Top Limit Switch", eTalon.GetSensorCollection().IsFwdLimitSwitchClosed());
	SmartDashboard::PutBoolean(name+" Bottom Limit Switch", eTalon.GetSensorCollection().IsRevLimitSwitchClosed());
	SmartDashboard::PutBoolean(name+" Slave Top Limit Switch", eSTalon.GetSensorCollection().IsFwdLimitSwitchClosed());
	SmartDashboard::PutBoolean(name+" Slave Bottom Limit Switch", eSTalon.GetSensorCollection().IsRevLimitSwitchClosed());

	SmartDashboard::PutNumber(name+" Closed Loop Error", eTalon.GetClosedLoopError(0));
	SmartDashboard::PutNumber(name+" Closed Loop Target", eTalon.GetClosedLoopTarget(0));
	SmartDashboard::PutNumber(name+" Bottom Limit Switch", eTalon.ConfigGetParameter(ctre::phoenix::ParamEnum::eProfileParamSlot_P,0,kTimeOut));

	SmartDashboard::PutNumber(name+" F",eTalon.ConfigGetParameter(ctre::phoenix::ParamEnum::eProfileParamSlot_F,0,10));
	SmartDashboard::PutNumber(name+" P",eTalon.ConfigGetParameter(ctre::phoenix::ParamEnum::eProfileParamSlot_P,0,10));
	SmartDashboard::PutNumber(name+" I",eTalon.ConfigGetParameter(ctre::phoenix::ParamEnum::eProfileParamSlot_I,0,10));
	SmartDashboard::PutNumber(name+" D",eTalon.ConfigGetParameter(ctre::phoenix::ParamEnum::eProfileParamSlot_D,0,10));

}
/*
 * This get height function returns the position that the robot is nearest to.
 */
double Elevator::GetHeight(EPos pos)
{
	return posVals[pos][0];//returns the position
}
/*
 * This get ramp function returns the ramp speed of the robot.
 */
double Elevator::GetRamp(EPos pos)
{
	return posVals[pos][1];//returns ramp
}
/*
 * This get max speed function returns the Speed of the robot.
 */
double Elevator::GetMaxSpeed(EPos pos)
{
	return posVals[pos][2];//returns speed
}

void Elevator::SetMaxRamp(double ramp)
{
	maxRamp = ramp;
}

//void eZeroed();
/*
 * The get EPos function returns the current raw elevator position.
 */
Elevator::EPos Elevator::GetEPos()
{
	return current;
}
/*
 * This Get Error function returns the difference from where we are
 * to where we are supposed to be
 */
double Elevator::GetError()
{
	return eTalon.GetClosedLoopError(0);//get from pid controller
}
/*
 * This Get Set Point function gets the position it is supposed to go to
 */
double Elevator::GetSetPoint()
{
	return eTalon.GetClosedLoopTarget(0);//where we are going
}
/*
 * This gets the raw encoder data for the Elevator
 */
double Elevator::GetEncoderPos()
{
	return eTalon.GetSelectedSensorPosition(kTimeOut);
}

void Elevator::SetEPos(EPos pos)
{
	current = pos;
	if(posVals[pos][0]<=eTalon.GetSelectedSensorPosition(0))
	{
		eTalon.SelectProfileSlot(1,0);
	}else
	{
		eTalon.SelectProfileSlot(0,0);
	}
	eTalon.Set(ControlMode::Position, posVals[pos][0]);
}
/*
 * This Function increments the positions until it gets to the top position.
 * It makes sure it doesn't go any higher.
 */
void Elevator::incPos()
{
	if(current!= EPos::Top)
	{
		current=(EPos)((int)current+1);
		SetEPos(current);
	}

}
/*
 * This function decrements the elevator positions all the way to the bottom
 * It makes sure it doesn't try to go any lower than the bottom position
 */
void Elevator::decPos()
{
	if(current!= EPos::Bottom)
	{
		current=(EPos)((int)current-1);
		SetEPos(current);
	}
}
/*
 * This function sets the Elevator motor speed
 */
void Elevator::SetMotorSpeed(double speed)
{
	eTalon.Set(ControlMode::PercentOutput,speed);
}

/*
 * This function doesn't allow the elevator to go higher than the top
 * position or any lower than the bottom position
 */
void Elevator::SetPosition(double pos)
{
	/*
	 * This if Function doesn't allow the elevator to go any higher than
	 * the top position
	 */
	if(pos>posVals[EPos::Top][0])
	{
		pos = posVals[EPos::Top][0];
	}
	/*
	 * This if function doesn't allow the elevator to go below the bottom
	 * position
	 */
	if(pos<posVals[EPos::Bottom][0])
	{
		pos = posVals[EPos::Bottom][0];
	}
	//eTalon.GetClosedLoopTarget(0);
	eTalon.Set(ControlMode::Position, pos);
}




