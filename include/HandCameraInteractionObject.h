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
#ifndef HANDCAMERAINTERACTIONOBJECT_H
#define HANDCAMERAINTERACTIONOBJECT_H

#include <math.h>
#include <std_msgs/Float64.h>

#include "ApplicationServices/MotorStateListener.h"
#include "UserCockpitConfiguration.h"
#include "Ros/RosWrapper.h"

#include "rose_main/interactive_image_display/interaction_object.hpp"
#include "rose_main/interactive_image_display/overlay/overlay_settings.hpp"
#include "rose_main/rose_main.hpp" //! @todo MdL: Remove dependency

namespace UCC = UserCockpitConfiguration;

class RoseMain;

template <typename T>
class HandCameraInteractionObject : public InteractionObject
{
public:
	typedef boost::shared_ptr<HandCameraInteractionObject> Ptr;

	struct HandCameraInteractionObjectDependencies
	{
        RoseMain * m_MainForm;
		boost::shared_ptr<CamController> m_HandCamController;
	    ApplicationServices::PublishingListener<const sensor_msgs::ImageConstPtr>::Ptr m_CameraLeftHandListener;
	    ApplicationServices::PublishingListener<const sensor_msgs::ImageConstPtr>::Ptr m_CameraRightHandListener;
		typename T::Ptr m_RosWrapper;
		UCC::UserCockpitConfigurationManager::Ptr m_CockpitConfiguration;
	};

	HandCameraInteractionObject( HandCameraInteractionObjectDependencies dependencies )
        : m_Dependencies(dependencies)
	{

	}

	~HandCameraInteractionObject()
	{
	    std::cout << " HandCameraInteractionObject::~HandCameraInteractionObject " << std::endl;
	}

	virtual void MoveStepUp()
	{
	}

	virtual void MoveUpStart()
	{
	}

	virtual void MoveUpStop()
	{
	}

	virtual void MoveStepDown()
	{
	}

	virtual void MoveDownStart()
	{
	}

	virtual void MoveDownStop()
	{
	}

	virtual void MoveStepLeft()
	{
	}

	virtual void MoveLeftStart()
	{
	}

	virtual void MoveLeftStop()
	{
	}

	virtual void MoveStepRight()
	{
	}

	virtual void MoveRightStart()
	{
	}

	virtual void MoveRightStop()
	{
	}

	virtual void MoveHome()
	{
	}

	virtual void SelectCam( LRCameraSelector cam )
	{
		if( cam == LeftCamera )
		{
			// LeftCamera
            m_Dependencies.m_CameraRightHandListener->Unsubscribe();
            
            m_Dependencies.m_CameraLeftHandListener->RegisterObserver( m_Dependencies.m_HandCamController );
            m_Dependencies.m_CameraLeftHandListener->Subscribe();
		}
		else
		{
			// RightCamera
            m_Dependencies.m_CameraLeftHandListener->Unsubscribe();
            
            m_Dependencies.m_CameraRightHandListener->RegisterObserver( m_Dependencies.m_HandCamController );
            m_Dependencies.m_CameraRightHandListener->Subscribe();
		}
        m_Dependencies.m_MainForm->selectHandCamera( cam );
	}
	
protected:
	
private:
	
	HandCameraInteractionObjectDependencies m_Dependencies;
};


#endif // HANDCAMERAINTERACTIONOBJECT_H
