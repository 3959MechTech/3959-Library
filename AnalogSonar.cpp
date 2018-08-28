/*
 * AnalogSonar.cpp
 *
 * Author: Brandon Fleming / Cole
 *
 * Date: FIRST Power Up 2018
 *
 * Description: This is the cpp file for the sensors on the robot that detect distance
 *
 * Summary: This file is short but is VERY important, it is fully documented for rookies to easily understand with knowledge given by Jim
 * it uses volts per inch ("0v to 5v") and 5" to 255" to detect distance.
 *
 * Change Log 8/27/18: Fully commented with new look and info
 *
 *
 */


#include "AnalogSonar.hpp"

/*
AnalogSonar::AnalogSonar(uint32_t channel)
{
	m_channel=channel;
	m_sensor = new AnalogInput(m_channel);
	m_allocatedChannel = true;
	init();

}

AnalogSonar::AnalogSonar(AnalogInput *channel)
{
	if(channel == NULL)
	{
		wpi_setWPIError(NullParameter);
	}else
	{
		m_sensor = channel;
		m_channel = m_sensor->GetChannel();
		init();
	}
	m_allocatedChannel = false;

}
*/

AnalogSonar::AnalogSonar(int channel) // constructor for the analog sonar
{
	m_sensor = new AnalogInput(channel); // starts the sensor
	m_channel = channel; // creates it as a channel
	init(); // initialize

	m_allocatedChannel = true;

}
AnalogSonar::~AnalogSonar()
{
	if(m_allocatedChannel)
	{
		delete m_sensor; // deletes sensor?
	}
}


void AnalogSonar::init()
{
	m_voltsPerInch = 0.0248046875; // minimum of volts per inch
//	LiveWindow::GetInstance()->AddSensor("AnalogSonar", m_sensor->GetChannel(), this);

	m_sensor->SetSampleRate(50000);
	m_sensor->SetAverageBits(5);
	m_sensor->SetOversampleBits(5);
}

float AnalogSonar::GetDistance()
{
	return (m_sensor->GetAverageVoltage())/m_voltsPerInch; // gets the distance by performing math with the volts and inches
}


double AnalogSonar::PIDGet()
{
	return GetDistance(); // returns the distance from the sensor
}









