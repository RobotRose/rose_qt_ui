//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================


// Customers.cpp
// Copyright Sioux 2010
// Initial Creation date: Sep 24, 2010
//
// Description: dependency passing to cockpit....


#ifndef DEPENDENCYINJECTOR_H_
#define DEPENDENCYINJECTOR_H_

#include <boost/shared_ptr.hpp>

#include "std_msgs/Bool.h"

// GuGu
#include "std_msgs/String.h"

#include "std_msgs/Int16.h"
#include "std_msgs/ByteMultiArray.h"
#include "std_msgs/Int8MultiArray.h"
#include "sensor_msgs/Image.h"
#include "rose_base_msgs/battery_state.h"

#include "ApplicationServices/Customers.h"
#include "ApplicationServices/PublishingListener.h"
#include "ApplicationServices/MotorStateListener.h"
 
#include "Ros/RosWrapper.h"

#include "UserCockpitConfiguration.h"
namespace UCC = UserCockpitConfiguration;

struct DependencyInjector
{
    typedef boost::shared_ptr<DependencyInjector> Ptr ;

	DependencyInjector(){};
    
    ApplicationServices::Customers::Ptr TSRCustomers;
    ApplicationServices::PublishingListener<const std_msgs::Bool::ConstPtr>::Ptr HeartBeatListener;
    ApplicationServices::PublishingListener<const std_msgs::String::ConstPtr>::Ptr LarmWarningListener;
    ApplicationServices::PublishingListener<const std_msgs::String::ConstPtr>::Ptr PlatformWarningListener;
    ApplicationServices::PublishingListener<const std_msgs::String::ConstPtr>::Ptr RarmWarningListener;
    ApplicationServices::PublishingListener<const rose_base_msgs::battery_state::ConstPtr>::Ptr BatteryMonitorListener;
    ApplicationServices::PublishingListener<const sensor_msgs::ImageConstPtr>::Ptr CameraOverviewListener;
    ApplicationServices::PublishingListener<const sensor_msgs::ImageConstPtr>::Ptr CameraLeftHandListener;
    ApplicationServices::PublishingListener<const sensor_msgs::ImageConstPtr>::Ptr CameraRightHandListener;
    ApplicationServices::PublishingListener<const sensor_msgs::ImageConstPtr>::Ptr CameraOperatorListener;
    ApplicationServices::PublishingListener<const std_msgs::ByteMultiArray::ConstPtr>::Ptr VoiceListener;
    ApplicationServices::PublishingListener<const dynamixel_msgs::JointState::ConstPtr>::Ptr PanMotorStateListener;
    ApplicationServices::PublishingListener<const dynamixel_msgs::JointState::ConstPtr>::Ptr TiltMotorStateListener;
    ApplicationServices::PublishingListener<const std_msgs::Int8MultiArray::ConstPtr>::Ptr SensorRingListener;

    UCC::UserCockpitConfigurationManager::Ptr CockpitConfiguration;

    Ros::RosWrapper::Ptr RosWrapper;
};


#endif /* DEPENDENCYINJECTOR_H_ */
