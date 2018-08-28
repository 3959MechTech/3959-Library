/*
 * AnalogSonar.hpp
 *
 * Author: Brandon Fleming / Cole
 *
 * Date: FIRST Power Up 2018
 *
 * Description: This is the hpp file for the sensors on the robot that detect distance
 *
 * Summary: This file is short but is VERY important, it is fully documented for rookies to easily understand with knowledge given by Jim
 * it uses volts per inch ("0v to 5v") and 5" to 255" to detect distance. This file holds the commands being used
 *
 * Change Log 8/27/18: Fully commented with new look and info
 *
 *
 */

#ifndef H_ANALOG_SONAR
#define H_ANALOG_SONAR

#include <WPILib.h> // getting all the includes being used
#include <LiveWindow/LiveWindowSendable.h> //smart dasboard sendable
#include <LiveWindow/LiveWindow.h> //hpp for the smart dashboard
#include <HAL/HAL.h>

#include <string.h> // includes string files

class AnalogSonar //: public SensorBase, public PIDSource, public LiveWindowSendable
{
private:
		AnalogInput *m_sensor; // the sensor on the bot
		int		 	m_channel; // the channel of the bot on thr sonar
        float		m_voltsPerInch; // the volts per inch (0v to 5v)
		float		m_DistanceOffset; // the distance calculated
		bool			m_allocatedChannel; // the channel allocated

public:
	//explicit DistanceSensor(uint32_t channel);
	// tom AnalogSonar(AnalogInput *channel);
	AnalogSonar(int  channel); // returning the channel of the sonar as a int
	virtual ~AnalogSonar(); // the signal from the sensor being known as analog sonar

	void 	init(); // initilizes

	double 	PIDGet(); // uses Calculus to make equations
	float 	GetDistance(); // gets the distance in inches
	float	GetDistanceOffset(){return m_DistanceOffset;}; // returnn distance offset
	int		GetSampleRate(){return m_sensor->GetSampleRate();};

	void		setDistanceOffset(float offset){m_DistanceOffset = offset;}
	void 	setVoltsPerInch(float vdp){m_voltsPerInch = vdp;}

	void 	UpdateTable();
	void 	StartLiveWindowMode(){}; // opens the live window
	void 	StopLiveWindowMode(){}; // starts the live window
	std::string GetSmartDashboardType(){return "AnalogInput";}; // finds the sonar name


		//static const float sensVolts = 0.009765625; // volts per Inch sensitivity


};



#endif /* H_ANALOG_SONAR */

