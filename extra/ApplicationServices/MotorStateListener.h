/*
 * MotorStateListener.h
 *
 *  Created on: Nov 8, 2010
 *      Author: martijn
 */

#ifndef MOTORSTATELISTENER_H_
#define MOTORSTATELISTENER_H_

#include "MotorState.h"
#include "ApplicationServices/ListenerImplementation.h"

namespace ApplicationServices
{

	class MotorStateListener : public MotorState, public ApplicationServices::ListenerImplementation< const dynamixel_msgs::JointState::ConstPtr >
	{
	public:
		typedef boost::shared_ptr<MotorStateListener> Ptr;
		MotorStateListener()
		{
		}
		virtual const dynamixel_msgs::JointState::ConstPtr GetLatestMotorState()
		{
			return m_state;
		}
		void CallbackImplementation(const dynamixel_msgs::JointState::ConstPtr & param)
		{
			m_state = param;
		}
	protected:
		//
		dynamixel_msgs::JointState::ConstPtr m_state;
		//
	};
};


#endif /* MOTORSTATELISTENER_H_ */
