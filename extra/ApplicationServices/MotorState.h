/*
 * MotorState.h
 *
 *  Created on: Nov 8, 2010
 *      Author: martijn
 */

#ifndef MOTORSTATE_H_
#define MOTORSTATE_H_

#include "dynamixel_msgs/JointState.h"
// #include "roscomm/joint_state.h"
#include <boost/shared_ptr.hpp>
using namespace boost;

class MotorState
{
public:
	typedef boost::shared_ptr<MotorState> Ptr;
	virtual const dynamixel_msgs::JointState::ConstPtr GetLatestMotorState() = 0;
};

#endif /* MOTORSTATE_H_ */
