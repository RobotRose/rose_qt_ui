/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/01/17
* 		- File created.
*
* Description:
*	description
* 
***********************************************************************************/
#ifndef OVERVIEWCAMERAINTERACTIONOBJECT_HPP
#define OVERVIEWCAMERAINTERACTIONOBJECT_HPP

#include "rose_main/interactive_image_display/interaction_object.hpp"

#include "ApplicationServices/MotorStateListener.h"
#include "Ros/RosWrapper.h"
#include <math.h>

#include <std_msgs/Float64.h>
//
#include "UserCockpitConfiguration.h"
namespace UCC = UserCockpitConfiguration;
//
template <typename T>
class OverviewCameraInteractionObject : public InteractionObject
{
public:
	typedef boost::shared_ptr<OverviewCameraInteractionObject> Ptr;

	struct OverviewCameraInteractionObjectDependencies
	{
		MotorState::Ptr m_PanMotorStateListener;
		MotorState::Ptr m_TiltMotorStateListener;
		typename T::Ptr m_RosWrapper;
		UCC::UserCockpitConfigurationManager::Ptr m_CockpitConfiguration;
	};

	OverviewCameraInteractionObject( OverviewCameraInteractionObjectDependencies dependencies )
		: m_Dependencies(dependencies)
	{
	}

	// ---------------------------------------
	//
	// ---------------------------------------	

	virtual void MoveStepUp()
	{
		PerformTiltStep( TiltDirectionUp );
	}
	//
	virtual void MoveUpStart()
	{
	}
	//
	virtual void MoveUpStop()
	{
	}
	//
	virtual void MoveStepDown()
	{
		PerformTiltStep( TiltDirectionDown );
	}
	//
	virtual void MoveDownStart()
	{
	}
	//
	virtual void MoveDownStop()
	{
	}
	//
	virtual void MoveStepLeft()
	{
		PerformPanStep( PanDirectionLeft );
	}
	//
	virtual void MoveLeftStart()
	{
	}
	//
	virtual void MoveLeftStop()
	{
	}
	//
	virtual void MoveStepRight()
	{
		PerformPanStep( PanDirectionRight );
	}
	//
	virtual void MoveRightStart()
	{
	}
	//
	virtual void MoveRightStop()
	{
	}
	//
	virtual void MoveHome()
	{
    	std_msgs::Float64 TiltHomePositionMessage;
		std_msgs::Float64 PanDefaultMessage;
		TiltHomePositionMessage.data = m_Dependencies.m_CockpitConfiguration->GetTiltMotorHomePosition();
		PanDefaultMessage.data = m_Dependencies.m_CockpitConfiguration->GetPanMotorHomePosition();
    	//
        m_Dependencies.m_RosWrapper->PublishToTopic( m_Dependencies.m_CockpitConfiguration->GetTiltMotorCommandTopic(), TiltHomePositionMessage );
		m_Dependencies.m_RosWrapper->PublishToTopic( m_Dependencies.m_CockpitConfiguration->GetPanMotorCommandTopic(), PanDefaultMessage );
	}
	//
	virtual void SelectCam( LRCameraSelector cam )
	{
	}
	//
protected:
	//
	enum TiltDirection { TiltDirectionUp = -1, TiltDirectionDown = 1 };
	void PerformTiltStep(TiltDirection direction )
	{
		dynamixel_msgs::JointState::ConstPtr latestState = m_Dependencies.m_TiltMotorStateListener->GetLatestMotorState();
                if(!latestState){
                    ROS_INFO("Unable to perform tilt: Could not retrieve latest motor state.");
                    return;
                }

		std_msgs::Float64 testMessage;
                double step = m_Dependencies.m_CockpitConfiguration->GetTiltStep();
                testMessage.data = latestState->current_pos - ((int)direction * step ); //! @todo MdL: - should be + ?!
		//
		m_Dependencies.m_RosWrapper->PublishToTopic( m_Dependencies.m_CockpitConfiguration->GetTiltMotorCommandTopic(), testMessage );
	}
	//
	enum PanDirection { PanDirectionRight = -1, PanDirectionLeft = 1 };
	void PerformPanStep( PanDirection direction )
	{
		dynamixel_msgs::JointState::ConstPtr latestState = m_Dependencies.m_PanMotorStateListener->GetLatestMotorState();
                if(!latestState){
                    ROS_INFO("Unable to perform pan: Could not retrieve latest motor state.");
                    return;
                }

		std_msgs::Float64 testMessage;
		double step = m_Dependencies.m_CockpitConfiguration->GetPanStep();
		testMessage.data = latestState->current_pos + ( int( direction ) * step );
		//
		m_Dependencies.m_RosWrapper->PublishToTopic( m_Dependencies.m_CockpitConfiguration->GetPanMotorCommandTopic(), testMessage );
	}
	//
private:
	//
	OverviewCameraInteractionObjectDependencies m_Dependencies;
};

#endif // OVERVIEWCAMERAINTERACTIONOBJECT_HPP
