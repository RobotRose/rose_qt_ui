/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*    Author: Mathijs de Langen
*    Date  : 2014/01/17
*         - File created.
*
* Description:
*    description
* 
***********************************************************************************/

#include <QtGui>
#include <QApplication>
#include "rose_startup/rose_startup.hpp"
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <ros/package.h>

#include <iostream>
#include <string>

#include "ros/ros.h"
#include "Ros/RosRunner.h"
#include "ApplicationServices/ThreadRunner.h"
#include "ApplicationServices/Customers.h"
#include "ApplicationServices/PublishingListener.h"

//GuGu
#include "std_msgs/String.h"
#include "std_msgs/ByteMultiArray.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Bool.h"

#include <boost/shared_ptr.hpp>

#include "SQLiteWrapper/SQLiteWrapper.h"

#include "roscomm/joint_state.h"
#include "sensor_msgs/Image.h"
#include "rose_base_msgs/battery_state.h"

#include "DependencyInjector.h"
//
#include "Loggers/LogMaster.h"

//
#include "Video/VideoCapture.h"
#include "Alsa/AlsaCapture.h"
//
#include "DataStore/DataStore/dataStoreIf.h"
#include "DataStore/DataStore/dataStore.h"
//
#include "UserCockpitConfiguration.h"

namespace UCC = UserCockpitConfiguration;
namespace AS = ApplicationServices;

#define TEST_CONFIG_FILE_DEFAULTS   (ros::package::getPath("rose_gui_application") + std::string("/config/UserCockpitDefault.xml"))
#define TEST_CONFIG_FILE_LOCAL      (ros::package::getPath("rose_gui_application") + std::string("/config/UserCockpitLocal.xml"))

//
//-------------------------------------------------------------------------------------------------
//

class UserCockpitSetup
{
public:
    UserCockpitSetup(int argc, char **argv)
        : m_argc(argc), m_argv(argv)
    {

    }

    int Run()
    {
        InitializeDataStore();

        InitializeCockpit();

        InitializeROS();

        SetupDependencies();

        StartRosService();

        StartPublishers();

        int returnValue = RunGUI();

        //delete m_DataStore.get();
        std::cout << Loggers::LogMaster::Instance()->ToString("[UserCockpit]");
        std::cout << "Bye :)" << std::endl ;

        return returnValue ;
    }
private:
    //
    void InitializeDataStore()
    {
        ROS_INFO("Opening datastore");
        m_DataStore = UCC::DataStoreIfPtr( new DataStore( "component" ) );
        m_DataStore->initialize( TEST_CONFIG_FILE_DEFAULTS, TEST_CONFIG_FILE_LOCAL );
        ROS_INFO("Closing datastore");

    }
        //
   void InitializeCockpit()
   {
        m_CockpitConfiguration = UCC::UserCockpitConfigurationManager::Ptr( new UCC::UserCockpitConfigurationManager( m_DataStore ) );
        m_CockpitConfiguration->NotifyConfiguration();
    }
    //
    void InitializeROS()
    {
        ros::init(m_argc, m_argv, m_CockpitConfiguration->GetUserCockpitNodeName(), ros::init_options::AnonymousName );

        m_RosWrapper = Ros::RosWrapper::Ptr(new Ros::RosWrapper());
        m_RosWrapper->SetQueueSize( 100 );
    }

    typedef const std_msgs::String::ConstPtr  StringPtr;
    typedef AS::PublishingListener<StringPtr> XStringPtr;

    typedef const std_msgs::Int16::ConstPtr Int16Ptr;
    typedef AS::PublishingListener<Int16Ptr> XInt16Ptr;

    typedef const rose_base_msgs::battery_state::ConstPtr BatteryStatePtr;
    typedef AS::PublishingListener<BatteryStatePtr> XBatteryStatePtr;

    typedef const sensor_msgs::ImageConstPtr  ImagePtr;
    typedef AS::PublishingListener<ImagePtr> XImagePtr;

    typedef const std_msgs::Bool::ConstPtr  BoolPtr;
    typedef AS::PublishingListener<BoolPtr> XBoolPtr;

    typedef const dynamixel_msgs::JointState::ConstPtr StatePtr;
    typedef AS::PublishingListener<StatePtr> XStatePtr;

    typedef const std_msgs::Int8MultiArray::ConstPtr   Int8ArrayPtr;
    typedef AS::PublishingListener<Int8ArrayPtr> XInt8ArrayPtr;

    typedef const std_msgs::ByteMultiArray::ConstPtr    ByteArrayPtr;
    typedef AS::PublishingListener<ByteArrayPtr> XByteArrayPtr;

    void SetupDependencies()
    {
       std::cout << "---------------------------------------------------------------------" << std::endl;
       std::cout << "----------- Begin of UserCockpitSetup::SetupDependencies ------------" << std::endl;
       std::cout << "---------------------------------------------------------------------" << std::endl;

        SQLiteWrapper::Ptr db( new SQLiteWrapper(GetDefaultLocationOfTheDatabaseFile()) );

        m_guiDependencyInjector = DependencyInjector::Ptr(new DependencyInjector());
        m_guiDependencyInjector->RosWrapper = m_RosWrapper;
        //
        m_guiDependencyInjector->TSRCustomers = AS::Customers::Ptr( new AS::Customers(db) );
        //
        m_guiDependencyInjector->BatteryMonitorListener = XBatteryStatePtr::Ptr( new XBatteryStatePtr( m_CockpitConfiguration->GetBatteryLevelTopicName(), m_RosWrapper)  );
        m_guiDependencyInjector->CameraLeftHandListener = XImagePtr::Ptr( new XImagePtr( m_CockpitConfiguration->GetLeftHandCamTopic(), m_RosWrapper) );
        m_guiDependencyInjector->CameraOverviewListener = XImagePtr::Ptr( new XImagePtr( m_CockpitConfiguration->GetOveriewCamTopic(), m_RosWrapper) );
        m_guiDependencyInjector->CameraRightHandListener = XImagePtr::Ptr( new XImagePtr( m_CockpitConfiguration->GetRightHandCamTopic(), m_RosWrapper) );

        m_guiDependencyInjector->CameraOperatorListener = XImagePtr::Ptr( new XImagePtr( m_CockpitConfiguration->GetOperatorCamTopic(), m_RosWrapper) );

        m_guiDependencyInjector->HeartBeatListener = XBoolPtr::Ptr( new XBoolPtr( m_CockpitConfiguration->GetHeartBeatTopic(), m_RosWrapper)  );
        m_guiDependencyInjector->LarmWarningListener = XStringPtr::Ptr( new XStringPtr( m_CockpitConfiguration->GetLarmWarningTopic(), m_RosWrapper)  );
        m_guiDependencyInjector->PanMotorStateListener = XStatePtr::Ptr( new XStatePtr( m_CockpitConfiguration->GetPanMotorStateTopic(), m_RosWrapper) );
        m_guiDependencyInjector->PlatformWarningListener = XStringPtr::Ptr( new XStringPtr( m_CockpitConfiguration->GetPlatformWarningTopic(), m_RosWrapper)  );
        m_guiDependencyInjector->RarmWarningListener = XStringPtr::Ptr( new XStringPtr( m_CockpitConfiguration->GetRarmWarningTopic(), m_RosWrapper)  );
        m_guiDependencyInjector->SensorRingListener = XInt8ArrayPtr::Ptr( new XInt8ArrayPtr( m_CockpitConfiguration->GetSensorRingTopic(), m_RosWrapper) );
        m_guiDependencyInjector->TiltMotorStateListener = XStatePtr::Ptr( new XStatePtr( m_CockpitConfiguration->GetTiltMotorStateTopic(), m_RosWrapper) );
        m_guiDependencyInjector->VoiceListener = XByteArrayPtr::Ptr( new XByteArrayPtr( m_CockpitConfiguration->GetCustomerVoiceTopic(), m_RosWrapper) );
        //
        m_guiDependencyInjector->CockpitConfiguration = m_CockpitConfiguration;
//        std::cout << m_CockpitConfiguration->GetOveriewCamTopic() << std::endl;
        std::cout << "---------------------------------------------------------------------" << std::endl;
        std::cout << "------------ End of UserCockpitSetup::SetupDependencies -------------" << std::endl;
        std::cout << "---------------------------------------------------------------------" << std::endl;

    }

    void StartRosService()
    {
        m_rosRunner = Ros::RosRunner::Ptr(new Ros::RosRunner());
        m_rosThreadRunner = AS::ThreadRunner::Ptr(new AS::ThreadRunner(m_rosRunner, "RosThreadRunner"));
    }

    int RunGUI()
    {
        m_application = boost::shared_ptr<QApplication>(new QApplication(m_argc,m_argv));

        m_roseStartup = RoseStartup::Ptr(new RoseStartup(m_guiDependencyInjector));
        m_roseStartup->show();

        return m_application->exec();
    }

    std::string GetDefaultLocationOfTheDatabaseFile()
    {
        // ROS_INFO( "Database name: %s", m_CockpitConfiguration->GetUserCockpitDatabaseName().c_str());
        return m_CockpitConfiguration->GetUserCockpitDatabaseName();
    }

    void StartPublishers()
    {
        ROS_DEBUG("UserCockpitSetup::StartPublishers::begin");
        // std::string OperatorCamDevice = m_CockpitConfiguration->GetOperatorCamDevice();
        // std::string OperatorCamTopic = m_CockpitConfiguration->GetOperatorCamTopic();
        // std::string OperatorVoiceTopic = m_CockpitConfiguration->GetOperatorVoiceTopic();
        
        // m_OperatorCamCapture = Video::VideoCapture::Ptr( new Video::VideoCapture( OperatorCamDevice, OperatorCamTopic, m_RosWrapper ) );
        // m_OperatorCamCaptureThread = AS::ThreadRunner::Ptr( new AS::ThreadRunner( m_OperatorCamCapture, "operatorCamCapture" ));
        
        // m_OperatorVoiceCapture = Alsa::AlsaCapture::Ptr( new Alsa::AlsaCapture( OperatorVoiceTopic, m_RosWrapper) );
        // m_OperatorVoiceCaptureThread = AS::ThreadRunner::Ptr( new AS::ThreadRunner( m_OperatorVoiceCapture, "operatorVoiceCapture" ));
        ROS_DEBUG("UserCockpitSetup::StartPublishers::end");
    }
    //
    UCC::DataStoreIfPtr m_DataStore;
    //
    // Video::VideoCapture::Ptr m_OperatorCamCapture;
    // AS::ThreadRunner::Ptr m_OperatorCamCaptureThread;
    //
    Alsa::AlsaCapture::Ptr m_OperatorVoiceCapture;
    AS::ThreadRunner::Ptr m_OperatorVoiceCaptureThread;
    //
    int m_argc;
    char **m_argv;
    DependencyInjector::Ptr m_guiDependencyInjector;
    Ros::RosWrapper::Ptr m_RosWrapper;
    Ros::RosRunner::Ptr m_rosRunner ;
    AS::ThreadRunner::Ptr m_rosThreadRunner ;
    boost::shared_ptr<QApplication> m_application;
    RoseStartup::Ptr m_roseStartup;

    UCC::UserCockpitConfigurationManager::Ptr m_CockpitConfiguration;
};
//
//-------------------------------------------------------------------------------------------------
//
int main(int argc, char **argv)
{
    UserCockpitSetup userCockpit(argc, argv) ;

    return userCockpit.Run();
}
//
//-------------------------------------------------------------------------------------------------
//
