/*
 * Velcocity Vector.hpp
 *
 *  Created on: Jan 27, 2018
 *      Author: Austin
 */

/* Author: Austin
 *
 * Date: 2018 Power Up
 *
 * Description:This code uses the type double to set the variables v and w equal to zero.
 *
 * Change Log: August 28, Code was further commented and the change log, along with the
 * description was added: Austin.
 *
 */
#ifndef VELOCITY_VECTOR_HPP_
#define VELOCITY_VECTOR_HPP_

/* Description: This is a vector that controls the velocity (speed) of the robot. */

class  VelocityVector
{
public:
	double v,w;// within this public class values v and w are set to 0 (w=Omega)
	VelocityVector(){v=0.0;w=0.0;}
};






#endif /* VELOCITY_VECTOR_HPP_ */
