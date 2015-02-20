//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

/*
 * UserCockpitConfiguration.h
 *
 *  Created on: Nov 18, 2010
 *      Author: martijn
 */

#ifndef USERCOCKPITCONFIGURATION_H_
#define USERCOCKPITCONFIGURATION_H_
//
#include <boost/shared_ptr.hpp>
using namespace boost;
//
#include <iostream>
#include <exception>
//
#include <ros/package.h>
// 
#include <string>
#include "DataStore/DataStore/dataStoreIf.h"
//
// ---------------------------------------------------------------------------------------------------------------------
//
namespace UserCockpitConfiguration
{
//
// -------------------------------------------------------------------------------------------------------------------
// typedef below prevents pollution of DataStore with Boost stuff
typedef boost::shared_ptr<DataStoreIf> DataStoreIfPtr;

class UserCockpitConfigurationManager
{
public:
	//
    class ItemDoesNotExistException : public std::exception
    {
    public:
        ItemDoesNotExistException(const std::string& componentName, const std::string& itemName)
        : m_itemName( itemName )
        , m_componentName( componentName )
        {

        }
        ~ItemDoesNotExistException() throw()
		{
		}
        const char* what() const throw()
        {
            return std::string( m_componentName + " : " + m_itemName ).c_str();
        }
    private:
        std::string m_itemName ;
        std::string m_componentName ;
    };

    //
    typedef boost::shared_ptr<UserCockpitConfigurationManager> Ptr;
    //
    UserCockpitConfigurationManager( DataStoreIfPtr datastore );
    //
    static std::string Component_PanMotor() { return "Head.PanMotor" ; }
    static std::string Component_TiltMotor() { return "Head.TiltMotor" ; }

    static std::string ItemName_MotorStepName() { return "Step" ; }
    static std::string ItemName_MotorStepMinName() { return "StepMin" ; }
    static std::string ItemName_MotorStepMaxName() { return "StepMax" ; }

    static std::string ItemName_MotorStateTopicName() { return "StateTopic"; };
    static std::string ItemName_MotorCommandTopicName() { return "CommandTopic"; };
    static std::string ItemName_MotorHomePosition() { return "HomePosition"; };

    virtual double GetPanStep();
    virtual std::string GetPanMotorStateTopic() throw (ItemDoesNotExistException) {return GetStringValue( Component_PanMotor(), ItemName_MotorStateTopicName() );  };
    virtual std::string GetPanMotorCommandTopic() throw (ItemDoesNotExistException) { return GetStringValue( Component_PanMotor(), ItemName_MotorCommandTopicName() ); };
    virtual double GetPanMotorHomePosition() throw (ItemDoesNotExistException) { return GetDoubleValue( Component_PanMotor(), ItemName_MotorHomePosition() ); };
    //
    virtual double GetTiltStep();
    virtual std::string GetTiltMotorStateTopic() throw (ItemDoesNotExistException) { return GetStringValue( Component_TiltMotor(), ItemName_MotorStateTopicName() ); };
    virtual std::string GetTiltMotorCommandTopic() throw (ItemDoesNotExistException) { return GetStringValue( Component_TiltMotor(), ItemName_MotorCommandTopicName() ); };
    virtual double GetTiltMotorHomePosition() throw (ItemDoesNotExistException) { return GetDoubleValue( Component_TiltMotor(), ItemName_MotorHomePosition() ); };
    //

    static std::string Component_UserCockpit() { return "UserCockpit"; };
    static std::string Component_OperatorCam() { return "Cameras.Operator"; };
    static std::string Component_OverviewCam() { return "Cameras.Overview"; };
    static std::string Component_LeftHandCam() { return "Cameras.LeftHand"; };
    static std::string Component_RightHandCam() { return "Cameras.RightHand"; };
    static std::string Component_HeartBeat() { return "HeartBeat"; };

//GuGu 1
    static std::string Component_LarmWarning() { return "Warning.Larm"; };
    static std::string Component_PlatformWarning() { return "Warning.Platform"; };
    static std::string Component_RarmWarning() { return "Warning.Rarm"; };

    static std::string Component_1ArmAutonomous() { return "1ArmAutonomous"; };
    static std::string Component_2ArmAutonomous() { return "2ArmAutonomous"; };

    static std::string Component_Button(std::string buttonNumber) { return "Button"+buttonNumber; };

    static std::string Component_OperatorVoice() { return "Voice.Operator"; };
    static std::string Component_CustomerVoice() { return "Voice.Customer"; };

    static std::string ItemName_UserCockpitNodeName() { return "RosNodeName"; };
    static std::string ItemName_UserCockpitDatabaseName() { return "CustomerDataBase"; };
    static std::string ItemName_DeviceName() { return "DeviceName"; };
    static std::string ItemName_TopicName() { return "Topic"; };
    static std::string ItemName_Enabled() { return "Enabled"; };

// GuGu 2
    static std::string ItemName_LabelName() { return "Label"; };
// FrHu
    static std::string ItemName_LabelValue() { return "Value"; };
    static std::string Component_BatteryMonitor() { return "BatteryMonitor"; };
    static std::string Component_BatteryMonitorLevel() { return "BatteryMonitor.Level"; };
    virtual std::string GetBatteryLevelTopicName() { return GetStringValue( Component_BatteryMonitorLevel(), ItemName_TopicName() );  };

    // Operator cam
    virtual std::string GetOperatorCamDevice() throw (ItemDoesNotExistException) { return GetStringValue( Component_OperatorCam(), ItemName_DeviceName() );  };
    virtual std::string GetOperatorCamTopic() throw (ItemDoesNotExistException) { return GetStringValue( Component_OperatorCam(), ItemName_TopicName() );  };
    virtual bool GetOperatorCamEnabled() throw (ItemDoesNotExistException) { return GetBoolValue( Component_OperatorCam(), ItemName_Enabled() );  };
    // Left hand cam
    virtual std::string GetLeftHandCamTopic() throw (ItemDoesNotExistException) { return GetStringValue( Component_LeftHandCam(), ItemName_TopicName() );  };
    // right hand cam
    virtual std::string GetRightHandCamTopic() throw (ItemDoesNotExistException) { return GetStringValue( Component_RightHandCam(), ItemName_TopicName() );  };

    virtual std::string GetOveriewCamTopic() throw (ItemDoesNotExistException) { return GetStringValue( Component_OverviewCam(), ItemName_TopicName() );  };


    // Operator Voice
    virtual std::string GetOperatorVoiceTopic() throw (ItemDoesNotExistException) { return GetStringValue( Component_OperatorVoice(), ItemName_TopicName() );  };
    // Customer Voice
    virtual std::string GetCustomerVoiceTopic() throw (ItemDoesNotExistException) { return GetStringValue( Component_CustomerVoice(), ItemName_TopicName() );  };
    // HeartBeat
    virtual std::string GetHeartBeatTopic() throw (ItemDoesNotExistException) { return GetStringValue( Component_HeartBeat(), ItemName_TopicName() );  };

//GuGu 3
    // LarmWarning
    virtual std::string GetLarmWarningTopic() throw (ItemDoesNotExistException) { return GetStringValue( Component_LarmWarning(), ItemName_TopicName() );  };
    // PlatformWarning
    virtual std::string GetPlatformWarningTopic() throw (ItemDoesNotExistException) { return GetStringValue( Component_PlatformWarning(), ItemName_TopicName() );  };
    // RarmWarning
    virtual std::string GetRarmWarningTopic() throw (ItemDoesNotExistException) { return GetStringValue( Component_RarmWarning(), ItemName_TopicName() );  };
    //1ArmAutonomous
    virtual std::string Get1ArmAutonomousTopic() throw (ItemDoesNotExistException) { return GetStringValue( Component_1ArmAutonomous(), ItemName_TopicName() );  };
    virtual std::string Get1ArmAutonomousButtonLabel(std::string buttonNumber) throw (ItemDoesNotExistException) { return GetStringValue( Component_1ArmAutonomous()+"."+Component_Button(buttonNumber), ItemName_LabelName() );  };
//FrHu
    virtual int Get1ArmAutonomousButtonValue(std::string buttonNumber) throw (ItemDoesNotExistException) { return GetIntValue( Component_1ArmAutonomous()+"."+Component_Button(buttonNumber), ItemName_LabelValue() );  };
//JaWi 1
    virtual bool Get1ArmAutonomousButtonEnabled(std::string buttonNumber) throw (ItemDoesNotExistException) { return GetBoolValue( Component_1ArmAutonomous()+"."+Component_Button(buttonNumber), ItemName_Enabled() );  };
    //2ArmAutonomous
    virtual std::string Get2ArmAutonomousTopic() throw (ItemDoesNotExistException) { return GetStringValue( Component_2ArmAutonomous(), ItemName_TopicName() );  };
    virtual std::string Get2ArmAutonomousButtonLabel(std::string buttonNumber) throw (ItemDoesNotExistException) { return GetStringValue( Component_2ArmAutonomous()+"."+Component_Button(buttonNumber), ItemName_LabelName() );  };
//FrHu
    virtual int Get2ArmAutonomousButtonValue(std::string buttonNumber) throw (ItemDoesNotExistException) { return GetIntValue( Component_2ArmAutonomous()+"."+Component_Button(buttonNumber), ItemName_LabelValue() );  };
    virtual bool Get2ArmAutonomousButtonEnabled(std::string buttonNumber) throw (ItemDoesNotExistException) { return GetBoolValue( Component_2ArmAutonomous()+"."+Component_Button(buttonNumber), ItemName_Enabled() );  };


    // UserCockpit
    virtual std::string GetUserCockpitNodeName() throw (ItemDoesNotExistException) { return GetStringValue( Component_UserCockpit(), ItemName_UserCockpitNodeName() );  };
    virtual std::string GetUserCockpitDatabaseName() throw (ItemDoesNotExistException) { return ros::package::getPath("rose_gui_application") + GetStringValue( Component_UserCockpit(), ItemName_UserCockpitDatabaseName() );  };
    //

    static std::string Component_SensorRing() { return "SensorRing" ; }

    virtual std::string GetSensorRingTopic() throw (ItemDoesNotExistException) { return GetStringValue( Component_SensorRing(), ItemName_TopicName() );  };

    static std::string ItemName_SensorCountLeft() { return "SensorCount_left"; };
    static std::string ItemName_SensorCountRight() { return "SensorCount_right"; };
    static std::string ItemName_SensorCountFront() { return "SensorCount_front"; };
    static std::string ItemName_SensorCountRear() { return "SensorCount_rear"; };

    virtual int GetSensorCountLeft() throw (ItemDoesNotExistException) { return GetIntValue( Component_SensorRing(), ItemName_SensorCountLeft() );  };
    virtual int GetSensorCountRight() throw (ItemDoesNotExistException) { return GetIntValue( Component_SensorRing(), ItemName_SensorCountRight() );  };
    virtual int GetSensorCountFront() throw (ItemDoesNotExistException) { return GetIntValue( Component_SensorRing(), ItemName_SensorCountFront() );  };
    virtual int GetSensorCountRear() throw (ItemDoesNotExistException) { return GetIntValue( Component_SensorRing(), ItemName_SensorCountRear() );  };

    static std::string ItemName_MinRange_cm() { return "MinRange_cm"; };
    static std::string ItemName_MaxRange_cm() { return "MaxRange_cm"; };

    virtual int GetSensorMinRange_cm() throw (ItemDoesNotExistException) { return GetIntValue( Component_SensorRing(), ItemName_MinRange_cm() );  };
    virtual int GetSensorMaxRange_cm() throw (ItemDoesNotExistException) { return GetIntValue( Component_SensorRing(), ItemName_MaxRange_cm() );  };

    void NotifyConfiguration();
    //
protected:
    //
    DataStoreIfPtr m_DataStore;
    //
private:
    //
    std::string GetStringValue( const std::string& componentName, const std::string& itemName );
    bool GetBoolValue( const std::string& componentName, const std::string& itemName );
    double GetDoubleValue( const std::string& componentName, const std::string& itemName );
    int GetIntValue( const std::string& componentName, const std::string& itemName );
    //
    double GetValue(const std::string& componentName, const std::string& itemName, const std::string& itemMinName, const std::string& itemMaxName) throw (ItemDoesNotExistException);
    //
    template <typename T>
    T ClipBoundaries(T value, T min, T max)
    {
        if (value > max)
        {
            return max;
        }
        else if (value < min)
        {
            return min;
        }
        return value;
    }
    //
    void NotifyConfigurationOfUserCockpit();
    void NotifyConfigurationOfHead();
    void NotifyConfigurationOfCameras();
    void NotifyConfigurationOfHeartBeat();

//GuGu 4
    void NotifyConfigurationOfLarmWarning();
    void NotifyConfigurationOfPlatformWarning();
    void NotifyConfigurationOfRarmWarning();
    void NotifyConfigurationOf1ArmAutonomous();
    void NotifyConfigurationOf2ArmAutonomous();
    void NotifyConfigurationOfSensorRing();
};

};

#endif /* USERCOCKPITCONFIGURATION_H_ */
