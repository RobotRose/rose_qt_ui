/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*    Author: Mathijs de Langen
*    Date  : 2013/11/22
*         - File created.
*
* Description:
*    The main cockpit window. This file contains the functionality of the main cockpit
*    window. 

//! @todo LvB: Put this attribution is a proper place
Icons for microphone and webcam are from:
Fugue Icons

(C) 2013 Yusuke Kamiyamane. All rights reserved.

These icons are licensed under a Creative Commons
Attribution 3.0 License.
<http://creativecommons.org/licenses/by/3.0/>

If you can't or don't want to provide attribution, please
purchase a royalty-free license.
<http://p.yusukekamiyamane.com/>

I'm unavailable for custom icon design work. But your
suggestions are always welcome!
<mailto:p@yusukekamiyamane.com>

* 
***********************************************************************************/
#ifndef ROSEMAIN_H
#define ROSEMAIN_H

#include <boost/shared_ptr.hpp>
#include <fstream>
#include <iostream>
#include <pwd.h>
#include <QtGui/QWidget>
#include <QtCore/QEvent>
#include <QtGui/QCloseEvent>
#include <QtCore/QTimer>
#include <QtGui/QtGui>
#include <ros/ros.h>
#include <sstream>
#include <stdio.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

//! @todo MdL: Fix unneeded includes and reorganize the rest
#include "Alsa/AlsaPlayback.h"
#include "ApplicationServices/ThreadRunner.h"
#include "ApplicationServices/MotorStateListener.h"
#include "Ros/RosWrapper.h"

#include "DependencyInjector.h"

#include "Led/Led.h"
#include "CLabel/CLabel.h"
#include "Battery/Battery.h"

#include "Controllers/SpeechVolumeController.h"
#include "Controllers/LedController.h"
#include "Controllers/CamController.h"
#include "Controllers/BatteryController.h"
#include "Controllers/LabelController.h"

#include "OverviewCameraInteractionObject.h"
#include "HandCameraInteractionObject.h"
#include "MapDisplayInteractionObject.h"

#include "GuiDefinitions.h"

#include "roscomm/toggle_operator_cam.h"
#include "roscomm/toggle_operator_voice_capturer.h"
#include "roscomm/toggle_customer_voice_capturer.h"
#include "roscomm/go_to_waypoint.h"
#include "roscomm/do_one_arm_autonomous.h"
#include "roscomm/do_two_arm_autonomous.h"
#include "rose_base_msgs/lift_state.h"
#include "rose_base_msgs/lift_command.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Empty.h"
#include "rose_joystick/switch_joystick_mode.h"

#include "rose_shared_variables/shared_variable.hpp"

#include "tf/transform_datatypes.h"

#include "Video/VideoCapture.h"

#include "Loggers/PlatformJoystickFileLogger.h"
#include "InteractiveImageDisplay/CameraDisplay.h"
#include "InteractiveImageDisplay/MapDisplay.h"
#include "InteractiveImageDisplay/OverviewCameraDisplay.h"
#include "SensorRing/SensorRingDisplay.h"
#include "TSRSlider/TSRSlider.h"
#include "Video/VideoCapture.h"
#include "Waypoint.h"

#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"

#include "connection_lost/connection_lost.hpp"
#include "operation_successful/operation_successful.hpp"

#include "rose_main/interactive_image_display/interaction_object.hpp"
#include "rose_main/interactive_image_display/overlay/overlay_settings.hpp"

#include "rose_main/ask_user_text_window/ask_user_text_window.hpp"
#include "rose_main/interaction_window/interaction_window.hpp"
#include "rose_main/manual_platform_control_window/manual_platform_control_window.hpp"
#include "rose_main/messages_window/messages_window.hpp"

#include "ui_RoseMain.h"

#define MAINQUIT_EVENT          ((QEvent::Type)(QEvent::User + 3))

class Battery;
class BatteryController;
class CamController;
class CLabel;
class CLed;
// class HandCameraInteractionObject;
class LabelController;
class LedController;
class SensorRingController;

using namespace std;
using namespace rose_shared_variables;

/**
 * Rosemain contains the main functionality of the Rose cockpit window. 
 */
class RoseMain : public QWidget
{
    Q_OBJECT

public:
    typedef boost::shared_ptr<RoseMain> Ptr;

    /**
     * Constructor
     * @param dependencies
     * @param customer
     * @param *parent 
     */
    RoseMain( boost::shared_ptr<DependencyInjector> dependencies, ApplicationServices::Customer customer, QWidget *parent = 0);

    /**
     * Destructor
     */
    ~RoseMain();

    /**
     * Selects which tab to show with this a certain camera
     * @param cam
     */
    void selectHandCamera( LRCameraSelector cam );

protected:
    //
private:
    /**
     * Initialize the user interface. All items in the interface are set up.
     */
    void InitGui();

    /**
     * Create new components that are placed on the user interface.
     */
    void CreateApplicationServicesComponents();

    /**
     * Register services and subcribe to provide interaction with the components.
     */
    void CreateConnectionsToApplicationServices();

    /**
     * Start the voice playback
     */
    void StartVoicePlayback();

    /**
     * Start operator camera
     */
    void StartOperatorCam();
    /**
     * Create the listeners for the neck position. 
     */
    void CreatePanAndTiltEndListeners();

    void closeEvent ( QCloseEvent * event ) ;
    bool event ( QEvent * pevent );
    
    //
    //                  VoiceListener
    //
    // We cannot use smart pointers for the objects that are passed to the layout objects using addWidget because
    // the layout takes ownership of this object and it will delete it.
    // If you see somewhere a bare pointer being used and you wonder why it is not a boost shared pointer
    // than the reason is most probably the same as the one described in this comment.
    //
    // Link to QT documentation: http://doc.trolltech.com/4.7-snapshot/qlayout.html#addItem

    /**
     * Creates the webcam pane to show the Overview camera image. This is the image from the kinect camera on Rose.
     */
    void CreateOverviewCamViewer();
    /**
     * Creates the webcam pane to show the Operator camera image. Is a child of the OverviewCamViewer.
     */
    void CreateOperatorCamViewer();
    void CreateHandCamViewer();
    void CreateMapDisplayViewer();
    //
    void CreateOverviewCamInteractionObject();
    void CreateHandCamInteractionObject();
    void CreateMapDisplayInteractionObject();
    //
    void LoadImagesForHandControl();
    //
    void AddAllSliders();

    void AddOperatorVoiceSlider();
    void AddClientVoiceSlider();
    //
    void AddHandOpenCloseSlider();
    void AddHandPressureSlider();

    void Configure1ArmAutonomousButtons();
    void Configure2ArmAutonomousButtons();
    
    void InitTimer();
    //
    void InitializeButtons();
    void AddBattery();
    void AddHeartBeatLed();
    void AddSensorRing();
    //
    void ReadWaypointsFile();
    void WriteWaypointsFile();
    void updatePoseCallback(nav_msgs::Odometry pose);

    void callWaypointService(Waypoint waypoint);
    void callOneArmAutonomousService(std::string name);
    void callTwoArmAutonomousService(std::string name);

    void initAutoNavServices();
    void setAutoNavEnabled(bool enable);

    void initPoseSubscription();
    void initInteractionWindow();
    void initAskUserTextWindow();
    void initMessagesWindow();
    void initManualPlatformControlWindow();
    void initJoystickModeSelection();

    void initCustomerInfo(ApplicationServices::Customer customer);

    void CB_checkOperationSuccessful();
    void CB_joystickmodesReceived(const roscomm::stringlist &modes);
    void CB_currentJoystickmodeReceived(const std_msgs::String &msg);
    void CB_emergency( const bool& emergency );
    void CB_changeStopButtonColor( const ros::TimerEvent& event );

    SharedVariable<bool>                sh_emergency_;
    bool                                stop_button_red_;
    TSRSlider* m_OperatorVoiceSlider;
    SliderSlave::Ptr m_OperatorVoiceSliderSlave;

    TSRSlider* m_ClientVoiceSlider;
    SliderSlave::Ptr m_ClientVoiceSliderSlave;

    TSRSlider* m_HandOpenCloseSlider;
    SliderSlave::Ptr m_HandOpenCloseSliderSlave;

    TSRSlider* m_HandPressureSlider;
    SliderSlave::Ptr m_HandPressureSliderSlave;

    //ExternalProcess::RosProcess::Ptr m_joystickProcess ;
    //Loggers::FileLogger::Ptr m_platformJoystickFileLogger ;
    //Loggers::PlatformJoystickFileLogger::Ptr m_platformJoystickLogger ;

    QTimer DisplayTimeTimer;
    time_t  m_ElapsedTime;
    ros::NodeHandle nodeHandle_;    //!< For advertising and subscribing.
    
    Ui::RoseMainClass ui;          //!< The user interface

    ApplicationServices::MotorStateListener::Ptr m_panMotorStateListener;
    ApplicationServices::MotorStateListener::Ptr m_tiltMotorStateListener;
    SensorRingDisplay::Ptr m_SensorRingDisplay;

    boost::shared_ptr<DependencyInjector> m_dependencies;
    boost::shared_ptr<CLed> m_HeartBeatLed;
    boost::shared_ptr<BatteryController> m_BatteryController;
    boost::shared_ptr<Battery> m_Battery;
    boost::shared_ptr<LedController> m_LedController;
    boost::shared_ptr<LabelController> m_LarmWarningLabelController;
    boost::shared_ptr<LabelController> m_PlatformWarningLabelController;
    boost::shared_ptr<LabelController> m_RarmWarningLabelController;
    boost::shared_ptr<CLabel> m_LarmWarningLabel;
    boost::shared_ptr<CLabel> m_PlatformWarningLabel;
    boost::shared_ptr<CLabel> m_RarmWarningLabel;
    boost::shared_ptr<SensorRingController> m_SensorRingController;
    boost::shared_ptr<InteractionObject> m_OverviewInteractionObject;
    boost::shared_ptr<InteractionObject> m_HandInteractionObject;
    boost::shared_ptr<InteractionObject> m_MapDisplayInteractionObject;
    boost::shared_ptr<CamController> m_HandCamController;
    boost::shared_ptr<CamController> m_OverviewCamController;
    boost::shared_ptr<CamController> m_OperatorCamController;
    //
    Alsa::AlsaPlayback::Ptr m_alsaPlayback;
    ApplicationServices::ThreadRunner::Ptr m_playbackThread;
    
    CameraDisplay* m_operatorCamera;
    CameraDisplay* m_cameraHand;
    CameraDisplay* m_cameraOverview;
    MapDisplay* m_MapDisplay;

    bool telepresenceIsOn;
    ros::ServiceClient operatorCameraClient;

    bool operatorVoiceIsOn;
    ros::ServiceClient operatorVoiceClient;

    bool customerVoiceIsOn;
    ros::ServiceClient customerVoiceClient;

    // Publishers
    void InitPublishers();
    ros::Publisher goal_publisher;
    ros::Publisher lift_publisher;
    ros::Publisher left_arm_select;

    ros::Publisher move_forwards_pub_;
    ros::Publisher move_backwards_pub_;
    ros::Publisher strafe_left_pub_;
    ros::Publisher stafe_right_pub_;
    ros::Publisher rotate_left_pub_;
    ros::Publisher rotate_right_pub_;
    ros::Publisher stop_pub_;
    ros::Publisher save_waypoint_pub_;
    
    ros::Publisher m_pushButton_2ArmAutonomous_clicked_publisher;
    ros::Publisher m_pushButton_1ArmAutonomous_clicked_publisher;

    ros::ServiceClient waypointServiceClient;
    ros::ServiceClient oneArmAutonomousServiceClient;
    ros::ServiceClient twoArmAutonomousServiceClient;
    std::list<Waypoint> waypointList;
    bool lastKnownPoseSet;
    bool operatorCameraAvailable_;
    nav_msgs::Odometry lastKnownPose;
    ros::Subscriber poseSubscriber;

    ros::Subscriber availableModesSubscriber;
    ros::Subscriber currentModeSubscriber;
    ros::ServiceClient modeSelectionClient;

    Video::VideoCapture::Ptr m_OperatorCamCapture;
    ApplicationServices::ThreadRunner::Ptr m_OperatorCamCaptureThread;
    
    InteractionWindow*              interaction_window_;
    AskUserTextWindow*              ask_user_text_window_;
    MessagesWindow*                 messages_window_;
    ManualPlatformControlWindow*    manual_platform_control_window_;

    LRCameraSelector current_hand_camera_;

    ros::NodeHandle n_;
    ros::Timer      change_emergency_button_colow_timer_;

Q_SIGNALS:
    void WaypointResponseReceived(bool waypointReached);
    void WaypointResponseError();

    void OneArmAutonomousResponseReceived(bool actionPerformed);
    void OneArmAutonomousResponseError();

    void TwoArmAutonomousResponseReceived(bool actionPerformed);
    void TwoArmAutonomousResponseError();

    //void SpeedChanged(double speed);

    void closed();

private Q_SLOTS:
    void DisplayTime();

    // Movement
    void on_moveForwardButton_clicked();
    void on_moveBackwardButton_clicked();
    void on_strafeLeftButton_clicked();
    void on_strafeRightButton_clicked();
    void on_rotateRightButton_clicked();
    void on_rotateLeftButton_clicked();
    void on_stopMovementButton_clicked();
    void on_m_ButtonStop_clicked();

    //Lift
    void on_liftSlider_sliderPressed();
    void on_liftSlider_sliderReleased();
    void on_liftSlider_valueChanged();
    void updateLiftSlider();
    //
    void on_m_operatorCameraImageChanged();
    void on_m_showOperatorCamCheckBox_clicked();
    //
    void on_noNavigationCheckbox_clicked();
    void on_navigationCheckbox_clicked();
    void on_posEstimateCheckbox_clicked();

    void on_noNavigationOrPoseSelection_selected();
    void on_PoseSelection_selected();
    void on_navigationGoalSelection_selected();
    void on_m_saveWaypointButton_clicked();

    void on_m_ButtonHelp_clicked();
    void on_m_ButtonClose_clicked();
    void on_m_buttonOperatorVolumeMute_clicked(bool state);
    void on_m_buttonClientVolumeMute_clicked(bool state);
    void on_m_buttonOperatorVideoMute_clicked(bool state);
    void on_m_buttonClientVideoMute_clicked(bool state);

    void on_m_ClosedGripperButton_clicked();
    void on_m_OpenedGripperButton_clicked();
    void on_m_JoystickModeSelector_currentIndexChanged(QString text);
};
//---------------------------------------------------------------------------------
//
//
#endif // ROSEMAIN_H

