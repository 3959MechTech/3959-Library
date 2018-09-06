/*
 * AnalogSonar.cpp
 *
 * Author: Brandon Fleming
 *
 * Date: FIRST Power Up 2018
 *
 * Description: This is the cpp file for the sensors on the robot that detect distance
 *
 * Summary: This file is short but is VERY important, it is fully documented for rookies to easily understand with knowledge given by Jim
 * it uses volts per inch ("0v to 5v") and 5" to 255" to detect distance.
 *
 * Change Log 8/27/18: Fully commented with new look and info along with description
 *
 *
 */


#include "AnalogSonar.hpp" // includes the commands in the analog sonar hpp file.

//COMMENTED OUT CODE IS NOT BEING USED!!

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

/*
this holds the basic info for the sonar and starts the sensor, init() starts the process
m_channel = channel creates it as a channel m_sensor = new AnalogInput(channel starts the 
sensor m_allocatedChannel = true sets it as true. m_channel sets the sonar as a channel.
*/
AnalogSonar::AnalogSonar(int channel)  // this is a constructor!!
{
	m_sensor = new AnalogInput(channel); 
	m_channel = channel; 
	init(); // initialize

	m_allocatedChannel = true;

}

/*
This calls the analog sonar, the if statement for the m_allocatedChannel deterimenes whether to delete the sensor or not.
the delete_sensor deletes the sensor.
*/
AnalogSonar::~AnalogSonar() 
{
	if(m_allocatedChannel) 
	{
		delete m_sensor; 
	}
}


/*
This initializes the sonar. The m_voltsPerInch is the minimum of volts per inch in the sonar. The commented out code 
is not being used.
*/
void AnalogSonar::init()
{
	m_voltsPerInch = 0.0248046875;
//	LiveWindow::GetInstance()->AddSensor("AnalogSonar", m_sensor->GetChannel(), this);
	
	m_sensor->SetSampleRate(50000);
	m_sensor->SetAverageBits(5);
	m_sensor->SetOversampleBits(5);
}

/*
This float is for the GetDistance() function which returns the inches detected by the sonar the return statement is 
getting the distance by performing math with the volts and inches.
*/
float AnalogSonar::GetDistance()
{
	return (m_sensor->GetAverageVoltage())/m_voltsPerInch;

double AnalogSonar::PIDGet() // PIDGet is calculus being used
{
	return GetDistance(); // returns the distance from the sensor
}










