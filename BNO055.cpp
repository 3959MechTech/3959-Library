/*
 * BNO055.cpp
 *
 *  Created on: Oct 28, 2017
 *      Author: Jim
 */

#include "BNO055.hpp"
#include <support/deprecated.h>
#include <support/mutex.h>
#include <WPILib.h>

/*
 * Description: This array contains cal data.
 *
 * IMPROVE!!  This should be improved to be changed from invoking
 */
const uint8_t BNO055::initCalData[calDataSize] =
{
	7,//0
	0,//1
	29,//2
	0,//3
	25,//4
	0,//5
	12,//6
	1,//7
	109,//8
	0,//9
	207,//10
	1,//11
	0,//12
	0,//13
	254,//14
	255,//15
	0,//16
	0,//17
	232,//18
	3,//19
	18,//20
	3//21
};

/*
 * Description: Constructor needs the I2C port on the RoboRio and the
 * 				address of the IMU.  The default is address A.  Soldering
 * 				the address pad of the board allows use of address B.
 */
BNO055::BNO055(I2C::Port port, int deviceAddress ):
	chip(port, deviceAddress),
	initVector(),
	currentVector()
{
	m_state = IMUState::INIT;
	adr = deviceAddress;

	cal = 0x00;

	for(int i = 0; i< calDataSize; i++)
	{
		calData[i]=0.0;
	}

}

/*
 * Description: Destructor
 */
BNO055::~BNO055()
{

}

/*
 * Description: Calls the appropriate function depending on state.
 */
void BNO055::Run()
{
	switch(m_state)
	{
	//if state is INIT, setup IMU and update state
	case IMUState::INIT: Begin();
						 m_state = IMUState::UPDATE;
						 break;
	//If state is in update, then update IMU vectors
	case IMUState::UPDATE: 	Update();
							break;

	}
	return;

}

/*
 * Description: Begin sets up the IMU for reading sensor and fusion data
 */
void BNO055::Begin()
{
	//drop into config mode
	SetOperationMode(BNO055::OPERATION_MODE_CONFIG);
	Wait(.02);//wait for mode change
	//set the use of the external oscillator
	chip.Write(BNO055::BNO055_SYS_TRIGGER_ADDR,0x80);
	//make sure we are in normal power mode.
	chip.Write(BNO055::BNO055_PWR_MODE_ADDR,BNO055::bno055_powermode_t::POWER_MODE_NORMAL);

	for(int i = 0; i<calDataSize;i++)
	{
		chip.Write(ACCEL_OFFSET_X_LSB_ADDR+i,initCalData[i]);
	}

	//chip.Write(BNO055::bno055_reg_t::BNO055_AXIS_MAP_SIGN_ADDR,0x01);//

	//set the mode to IMU with fusion engine
	chip.Write(BNO055::BNO055_OPR_MODE_ADDR,BNO055::bno055_opmode_t::OPERATION_MODE_IMUPLUS);
	Wait(.02);//wait for mode change

	initVector = GetQuat();//init heading

}

/*
 * Description: Use this method to set the mode of the IMU
 */
void BNO055::SetOperationMode(BNO055::bno055_opmode_t mode)
{
	chip.Write(BNO055::BNO055_OPR_MODE_ADDR, mode);
}

/*
 * Description: Get current status of the chip
 */
BNO055::bno055_status_t BNO055::GetStatus()
{
	uint8_t data;
	chip.Read(BNO055::BNO055_SYS_STAT_ADDR,1,&data);
	return (bno055_status_t)data;
}

/*
 * Description: Read Chip Error Register
 */
uint8_t BNO055::GetError()
{
	uint8_t data;
	chip.Read(BNO055::BNO055_SYS_ERR_ADDR,1,&data);
	return (double)data;
}

/*
 * Description: Read Chip ID, it's like a serial number
 */
double BNO055::GetChipID()
{
	uint8_t data;
	chip.Read(BNO055::BNO055_CHIP_ID_ADDR,1,&data);
	return (double)data;
}

/*
 * Description: Gets the mode of the IMU
 */
BNO055::bno055_opmode_t BNO055::GetMode()
{
	uint8_t data;
	chip.Read(BNO055::BNO055_OPR_MODE_ADDR,1,&data);
	return (bno055_opmode_t)data;
}

/*
 * Description: Read data from I2C bus.
 * Parameters:
 * 		reg = the address or starting address you want to read
 * 		Len = how many bytes you want to read.
 * 		*data = a pointer to a byte array/buffer that is at least
 * 				the size of Len.
 */
int BNO055::Read(uint8_t reg, int Len, uint8_t *data)
{
	return (int)chip.Read(reg,Len,data);
}

/*
 * Description: Write 1 byte of data on I2C bus.
 * Parameters:
 * 		reg = the address or starting address write to
 * 		data = byte of data to store
 */
int BNO055::Write(uint8_t reg, uint8_t data)
{
	return (int)chip.Write(reg,data);
}

/*
 * Description: This returns the chip temperature
 */
double BNO055::GetTemp(void)
{
	uint8_t data;
	Read(BNO055_TEMP_ADDR,1,&data);

	return (double)data;
}

/*
 * Description: Get the status of calibration
 */
uint8_t BNO055::GetCalStatus()
{
	uint8_t data;
	Read(BNO055_CALIB_STAT_ADDR,1,&data);
	return data;
}

/*
 * Description: Get calibration data/results
 */
void BNO055::GetCalData()
{
	uint8_t data[calDataSize];
	Read(ACCEL_OFFSET_X_LSB_ADDR,calDataSize, data);

	for(int i = 0; i<calDataSize; i++)
	{
		calData[i]=(double)data[i];
	}

}

/*
 * Description: Get Quaternion from chip
 */
Quaternion BNO055::GetQuat(void)
{
  uint8_t buffer[8];
  memset (buffer, 0, 8);

  int16_t x, y, z, w;
  x = y = z = w = 0;

  /* Read quat data (8 bytes) */
  Read(BNO055_QUATERNION_DATA_W_LSB_ADDR,8, buffer);
  w = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
  x = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
  y = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
  z = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);

  /* Assign to Quaternion */
  /* See http://ae-bst.resource.bosch.com/media/products/dokumente/bno055/BST_BNO055_DS000_12~1.pdf
     3.6.5.5 Orientation (Quaternion)  */
  const double scale = (1.0 / (1<<14));

  Quaternion quat(scale * w, scale * x, scale * y, scale * z);

  return quat;
}

/*
 * Description: Get 3D vector based on vector type
 */
Vector BNO055::GetVector(vector_type_t type)
{
	uint8_t data[6];
	Vector v;

	int16_t x=0,y=0,z=0;

	Read(type, 6, data);

	x = ((int16_t)data[0]) | (((int16_t)data[1]) << 8);
	y = ((int16_t)data[2]) | (((int16_t)data[3]) << 8);
	z = ((int16_t)data[4]) | (((int16_t)data[5]) << 8);

	/* Convert the value to an appropriate range (section 3.6.4) */
	  /* and assign the value to the Vector type */
	  switch(type)
	  {
	    case VECTOR_MAGNETOMETER:
	      /* 1uT = 16 LSB */
	      v.x = ((double)x)/16.0;
	      v.y = ((double)y)/16.0;
	      v.z = ((double)z)/16.0;
	      break;
	    case VECTOR_GYROSCOPE:
	      /* 1dps = 16 LSB */
	    	  v.x = ((double)x)/16.0;
	    	  v.y = ((double)y)/16.0;
	      v.z = ((double)z)/16.0;
	      break;
	    case VECTOR_EULER:
	      /* 1 degree = 16 LSB */
	    	  v.x = ((double)x)/16.0;
	    	  v.y = ((double)y)/16.0;
	      v.z = ((double)z)/16.0;
	      break;
	    case VECTOR_ACCELEROMETER:
	    case VECTOR_LINEARACCEL:
	    case VECTOR_GRAVITY:
	      /* 1m/s^2 = 100 LSB */
	    	  v.x = ((double)x)/100.0;
	    	  v.y = ((double)y)/100.0;
	    	  v.z = ((double)z)/100.0;
	      break;
	  }

	return v;
}

/*
 * Description: Update local quaternion vector
 */
void BNO055::UpdateQuaternion()
{
	currentVector = GetQuat();
}

/*
 * Description: Gets the current quaternion and set it
 * 				as the "zero" point.
 */
void BNO055::SetCurrentQuaternionAsInit()
{
	initVector = currentVector;
}

/*
 * Description: Get current Quaternion
 */
double BNO055::GetCurrentQuaternion(QuaternionElement element)
{
	switch(element)
	{
	case QuaternionElement::X : return currentVector.GetX();
	case QuaternionElement::Y : return currentVector.GetY();
	case QuaternionElement::Z : return currentVector.GetZ();
	case QuaternionElement::W : return currentVector.GetW();
	default: return 0.0;
	}
}

/*
 * Description: Get the current "zero" quaternion
 */
double BNO055::GetInitQuaternion(QuaternionElement element)
{
	switch(element)
	{
	case QuaternionElement::X : return initVector.GetX();
	case QuaternionElement::Y : return initVector.GetY();
	case QuaternionElement::Z : return initVector.GetZ();
	case QuaternionElement::W : return initVector.GetW();
	default: return 0.0;
	}
}

/*
 * Description: Update all the vectors and status
 *
 * IMPROVE!!  Set up system where only requested vectors are updated
 */
void BNO055::Update()
{
	//BNO055_SYS_STAT_ADDR
	//BNO055_SYS_ERR_ADDR
	if( GetMode() == BNO055::OPERATION_MODE_CONFIG)
	{
		cal = GetCalStatus();
		GetCalData();
	}else
	{
		//cal = GetCalStatus();
		currentVector = GetQuat();
		eulerAngles = GetVector(VECTOR_EULER);
		mag = GetVector(VECTOR_MAGNETOMETER);
		gyro = GetVector(VECTOR_GYROSCOPE);
		acc = GetVector(VECTOR_ACCELEROMETER);
		gravity = GetVector(VECTOR_GRAVITY);

	}

}

/*
 * Description: Send all data to Smart Dashboard.
 * 				The name passed in will be added to the data labels.
 */
void BNO055::SendSMData(std::string name="IMU")
{

	char buffer[100];
	uint8_t data = GetError();
	std::sprintf(buffer,"_0x%x",data);
	SmartDashboard::PutString(name+"_Mode", BNO055_Mode_String(GetMode()));
	SmartDashboard::PutString(name+"_Status", BNO055_Status_String(GetStatus()));
	SmartDashboard::PutString(name+"_Error", buffer);
	SmartDashboard::PutNumber(name+"_initVectorW", initVector.GetW());
	SmartDashboard::PutNumber(name+"_initVectorX", initVector.GetX());
	SmartDashboard::PutNumber(name+"_initVectorY", initVector.GetY());
	SmartDashboard::PutNumber(name+"_initVectorZ", initVector.GetZ());

	SmartDashboard::PutNumber(name+"_CurrentVectorW", currentVector.GetW());
	SmartDashboard::PutNumber(name+"_CurrentVectorX", currentVector.GetX());
	SmartDashboard::PutNumber(name+"_CurrentVectorY", currentVector.GetY());
	SmartDashboard::PutNumber(name+"_CurrentVectorZ", currentVector.GetZ());

	SmartDashboard::PutNumber(name+"_RelativeHeading", initVector.GetAngleBetween(initVector, currentVector));

	SmartDashboard::PutNumber(name+"_Temp_C", GetTemp());

	SmartDashboard::PutNumber(name+"_CalStatus", GetCalStatus());
	SmartDashboard::PutNumber(name+"_CalStatus_Sys", GetCalSys());
	SmartDashboard::PutNumber(name+"_CalStatus_Gyro", GetCalGryo());
	SmartDashboard::PutNumber(name+"_CalStatus_Acc", GetCalAcc());
	SmartDashboard::PutNumber(name+"_CalStatus_Mag", GetCalMag());

	//SmartDashboard::PutNumberArray(name+"_CalData",calData);
	char reg[25];
	for(int i = 0;i<calDataSize;i++)
	{
		std::sprintf(reg,"_0x%x",ACCEL_OFFSET_X_LSB_ADDR+i);
		SmartDashboard::PutNumber(name+std::string(reg), calData[i]);
	}

	eulerAngles.SendSDData(name+"_eulerAngle");
	mag.SendSDData(name+"_Mag");
	gyro.SendSDData(name+"_Gyro");
	acc.SendSDData(name+"_Acc");
	gravity.SendSDData(name+"_gravity");


}
