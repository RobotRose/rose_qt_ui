//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

/*
 * UserCockpitConfiguration.cpp
 *
 *  Created on: Nov 18, 2010
 *      Author: martijn
 */

#include "UserCockpitConfiguration.h"
#include "UnitConverter/UnitConverter.h"
#include <string>
#include <iostream>
using namespace UserCockpitConfiguration;
//
//----------------------------------------------------------------------------------------------------------
//
UserCockpitConfigurationManager::UserCockpitConfigurationManager( DataStoreIfPtr datastore )
:m_DataStore( datastore )
{
}
//
//----------------------------------------------------------------------------------------------------------
//
double UserCockpitConfigurationManager::GetPanStep()
{
    return UnitConverter::DegreesToRadians( GetValue( Component_PanMotor(), ItemName_MotorStepName(), ItemName_MotorStepMinName(), ItemName_MotorStepMaxName() ) );
}
//
//----------------------------------------------------------------------------------------------------------
//
double UserCockpitConfigurationManager::GetTiltStep()
{
    return UnitConverter::DegreesToRadians( GetValue( Component_TiltMotor(), ItemName_MotorStepName(), ItemName_MotorStepMinName(), ItemName_MotorStepMaxName() ) );
}
//
//----------------------------------------------------------------------------------------------------------
//
double UserCockpitConfigurationManager::GetValue(const std::string& componentName, const std::string& itemName, const std::string& itemMinName, const std::string& itemMaxName) throw (ItemDoesNotExistException)
{
    bool result = true;
    double min, max, val;

    result = m_DataStore->getItemValue( componentName, itemName, val );
    if (!result)
    {
        throw ItemDoesNotExistException(componentName, itemName);
    }

    result = m_DataStore->getItemValue( componentName, itemMinName, min );
    if (!result)
    {
        throw ItemDoesNotExistException(componentName, itemName);
    }

    result = m_DataStore->getItemValue( componentName, itemMaxName, max );
    if (!result)
    {
        throw ItemDoesNotExistException(componentName, itemName);
    }

    return ClipBoundaries(val, min, max);
}
//
//----------------------------------------------------------------------------------------------------------
//
std::string UserCockpitConfigurationManager::GetStringValue( const std::string& componentName, const std::string& itemName )
{
    bool result = true;
    std::string tmp = "";

    result = m_DataStore->getItemValue( componentName, itemName, tmp );
    if (!result)
    {
        throw ItemDoesNotExistException(componentName, itemName);
    }
    return tmp;
}
//
//----------------------------------------------------------------------------------------------------------
//
bool UserCockpitConfigurationManager::GetBoolValue( const std::string& componentName, const std::string& itemName )
{
    bool result = true;
    bool tmp = false;

    result = m_DataStore->getItemValue( componentName, itemName, tmp );
    if (!result)
    {
        throw ItemDoesNotExistException(componentName, itemName);
    }
    return tmp;
}
//
//----------------------------------------------------------------------------------------------------------
//
double UserCockpitConfigurationManager::GetDoubleValue( const std::string& componentName, const std::string& itemName )
{
    bool result = true;
    double tmp = false;

    result = m_DataStore->getItemValue( componentName, itemName, tmp );
    if (!result)
    {
        throw ItemDoesNotExistException(componentName, itemName);
    }
    return tmp;
}
//
//----------------------------------------------------------------------------------------------------------
//
int UserCockpitConfigurationManager::GetIntValue( const std::string& componentName, const std::string& itemName )
{
    bool result = true;
    int tmp = 0;

    result = m_DataStore->getItemValue( componentName, itemName, tmp );
    if (!result)
    {
        throw ItemDoesNotExistException(componentName, itemName);
    }
    return tmp;
}
//
//----------------------------------------------------------------------------------------------------------
//
void UserCockpitConfigurationManager::NotifyConfiguration()
{
	std::cout << "---------------------------------------------------------------------" << std::endl;
	std::cout << "--- Start of UserCockpitConfigurationManager::NotifyConfiguration ---" << std::endl;
	std::cout << "---------------------------------------------------------------------" << std::endl;

	NotifyConfigurationOfUserCockpit();
	NotifyConfigurationOfHead();
	NotifyConfigurationOfCameras();
	NotifyConfigurationOfHeartBeat();
	NotifyConfigurationOfSensorRing();
    NotifyConfigurationOf1ArmAutonomous();
    NotifyConfigurationOf2ArmAutonomous();

	std::cout << "---------------------------------------------------------------------" << std::endl;
	std::cout << "---- End of UserCockpitConfigurationManager::NotifyConfiguration ----" << std::endl;
	std::cout << "---------------------------------------------------------------------" << std::endl;
}
//
//----------------------------------------------------------------------------------------------------------
//
void UserCockpitConfigurationManager::NotifyConfigurationOfUserCockpit()
{
	std::cout << "General UserCockpit configuration" << std::endl;
	std::cout << "  Node name : " << GetUserCockpitNodeName() << std::endl;
	std::cout << "  Database name : " << GetUserCockpitDatabaseName() << std::endl;
}
//
//----------------------------------------------------------------------------------------------------------
//
void UserCockpitConfigurationManager::NotifyConfigurationOfHead()
{
	std::cout << "Head configuration" << std::endl;
	std::cout << "  PanMotor" << std::endl;
	std::cout << "    Step : " << GetPanStep() << std::endl;
	std::cout << "    State topic : " << GetPanMotorStateTopic() << std::endl;
	std::cout << "    Command topic : " << GetPanMotorCommandTopic() << std::endl;
	std::cout << "    Home position : " << GetPanMotorHomePosition() << std::endl;
	std::cout << "  TiltMotor" << std::endl;
	std::cout << "    Step  : " << GetTiltStep() << std::endl;
	std::cout << "    State topic : " << GetTiltMotorStateTopic() << std::endl;
	std::cout << "    Command topic : " << GetTiltMotorCommandTopic() << std::endl;
	std::cout << "    Home position : " << GetTiltMotorHomePosition() << std::endl;
}//
//----------------------------------------------------------------------------------------------------------
//
void UserCockpitConfigurationManager::NotifyConfigurationOfCameras()
{
	std::cout << "Cameras" << std::endl;
	std::cout << "  Operator" << std::endl;
	std::cout << "    Topic   : " << GetOperatorCamTopic() << std::endl;
	std::cout << "  Left hand" << std::endl;
	std::cout << "    Topic   : " << GetLeftHandCamTopic() << std::endl;
	std::cout << "  Right hand" << std::endl;
	std::cout << "    Topic   : " << GetRightHandCamTopic() << std::endl;
}
//
//----------------------------------------------------------------------------------------------------------
//
void UserCockpitConfigurationManager::NotifyConfigurationOfHeartBeat()
{
	std::cout << "HeartBeat" << std::endl;
	std::cout << "  Topic   : " << GetHeartBeatTopic() << std::endl;
}

//GuGu 5
//
//----------------------------------------------------------------------------------------------------------
//
void UserCockpitConfigurationManager::NotifyConfigurationOfLarmWarning()
{
        std::cout << "UserCockpitConfigurationManager::LarmWarning" << std::endl;
        std::cout << "  Topic   : " << GetLarmWarningTopic() << std::endl;
}

//GuGu 5
//
//----------------------------------------------------------------------------------------------------------
//
void UserCockpitConfigurationManager::NotifyConfigurationOfPlatformWarning()
{
        std::cout << "UserCockpitConfigurationManager::PlatformWarning" << std::endl;
        std::cout << "  Topic   : " << GetPlatformWarningTopic() << std::endl;
}

//GuGu 5
//
//----------------------------------------------------------------------------------------------------------
//
void UserCockpitConfigurationManager::NotifyConfigurationOfRarmWarning()
{
        std::cout << "UserCockpitConfigurationManager::RarmWarning" << std::endl;
        std::cout << "  Topic   : " << GetRarmWarningTopic() << std::endl;
}

//GuGu 5
//
//----------------------------------------------------------------------------------------------------------
//
void UserCockpitConfigurationManager::NotifyConfigurationOf1ArmAutonomous()
{
        std::cout << "UserCockpitConfigurationManager::1ArmAutonomous" << std::endl;
//        std::cout << "  Button1 Label  : " << Get1ArmAutonomousButton1Label() << std::endl;
        std::cout << "  Button1 Topic  : " << Get1ArmAutonomousTopic() << std::endl;
}

//JaWi2
//----------------------------------------------------------------------------------------------------------
//
void UserCockpitConfigurationManager::NotifyConfigurationOf2ArmAutonomous()
{
        std::cout << "UserCockpitConfigurationManager::2ArmAutonomous" << std::endl;
//        std::cout << "  Button1 Label  : " << Get2ArmAutonomousButton1Label() << std::endl;
        std::cout << "  Button1 Topic  : " << Get2ArmAutonomousTopic() << std::endl;
}

//
//----------------------------------------------------------------------------------------------------------
//
void UserCockpitConfigurationManager::NotifyConfigurationOfSensorRing()
{
	std::cout << "SensorRing" << std::endl;
	std::cout << "  Topic   : " << GetSensorRingTopic() << std::endl;
	std::cout << "  Sensor count left : " << GetSensorCountLeft() << std::endl;
	std::cout << "  Sensor count right : " << GetSensorCountRight() << std::endl;
	std::cout << "  Sensor count front : " << GetSensorCountFront() << std::endl;
	std::cout << "  Sensor count read : " << GetSensorCountRear() << std::endl;
	std::cout << "  Min range cm : " << GetSensorMinRange_cm() << std::endl;
	std::cout << "  Max range cm : " << GetSensorMaxRange_cm() << std::endl;
}
//
//----------------------------------------------------------------------------------------------------------
//
