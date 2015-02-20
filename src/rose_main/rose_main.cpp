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
* 
***********************************************************************************/
#include "rose_main/rose_main.hpp"

RoseMain::RoseMain( DependencyInjector::Ptr dependencies, ApplicationServices::Customer customer, QWidget *parent)
    : QWidget(parent)
    , m_dependencies         ( dependencies )
    , current_hand_camera_   ( RightCamera )
    , n_ ( "rose_main")
    , sh_emergency_(SharedVariable<bool>("emergency"))
    , stop_button_red_ ( true )
{
    ROS_DEBUG("RoseMain::RoseMain::begin");
    ui.setupUi(this);

    nodeHandle_ = ros::NodeHandle();
    //
    InitGui();
    //
    //StartVoicePlayback();
    StartOperatorCam();
    //
    InitTimer();
    //
    CreateApplicationServicesComponents();
    //
    CreateConnectionsToApplicationServices();

    //m_platformJoystickFileLogger = Loggers::FileLogger::Ptr(new Loggers::FileLogger("~/.UserCockpit/PlatformJoystick.log"));
    //m_platformJoystickLogger = Loggers::PlatformJoystickFileLogger::Ptr(new Loggers::PlatformJoystickFileLogger(m_platformJoystickFileLogger));
    //m_joystickProcess = ExternalProcess::RosProcess::Start("rosrun PlatformJoystick PlatformJoystick",m_platformJoystickLogger);

    InitPublishers();

    ReadWaypointsFile();
    initPoseSubscription();
    initCustomerInfo(customer);
    initInteractionWindow();
    initAskUserTextWindow();
    initMessagesWindow();
    initManualPlatformControlWindow();
    initJoystickModeSelection();
    //setWindowFlags(Qt::FramelessWindowHint | Qt::Popup);
    //showFullScreen();
    this->showMaximized();

    setStyleSheet(TsrDefinitions::Instance()->GetStyleSheet().c_str());

    // Connect to state of emercency
    sh_emergency_.connect(ros::Duration(0.1));
    sh_emergency_.registerChangeCallback(boost::bind(&RoseMain::CB_emergency, this, _1));
    CB_emergency(sh_emergency_);

    change_emergency_button_colow_timer_ = nodeHandle_.createTimer(ros::Duration(0.5), &RoseMain::CB_changeStopButtonColor, this, false, false);

    ROS_DEBUG("RoseMain::RoseMain::end");
}
//---------------------------------------------------------------------------------
//
//
RoseMain::~RoseMain()
{
    std::cout << __FILE__ << " : " << __LINE__ << " : " << " RoseMain::~RoseMain m_ElapsedTime:" << m_ElapsedTime << std::endl;
    DisplayTimeTimer.stop();

    WriteWaypointsFile();

    //m_joystickProcess->Stop();
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::initCustomerInfo(ApplicationServices::Customer customer)
{
    std::vector<unsigned char> picData = customer.GetPicture();
    QPixmap pixmap;
    pixmap.loadFromData(&picData[0], picData.size());
    ui.labelClientName->setText(customer.GetName().c_str());
    ui.labelClientImage->setScaledContents(true);
    ui.labelClientImage->setPixmap(pixmap);
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::ReadWaypointsFile()
{

}
//---------------------------------------------------------------------------------
//
//
void RoseMain::WriteWaypointsFile()
{
  
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::selectHandCamera( LRCameraSelector cam )
{
    current_hand_camera_ = cam;
    //! @todo MdL: Change slider values of the current cam
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::InitTimer()
{
    //    DisplayTimeTimer.setInterval ( 1000 );
    DisplayTimeTimer.setInterval ( 200 );       // 2011/03/30    raise the update frequency for central update experiment
    connect( &DisplayTimeTimer, SIGNAL( timeout() ), this, SLOT( DisplayTime() ) );
    DisplayTimeTimer.start();
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::initInteractionWindow()
{
    interaction_window_ = new InteractionWindow( NULL );
    ui.m_interactionWidget->layout()->addWidget( interaction_window_ );
};
//---------------------------------------------------------------------------------
//
//
void RoseMain::initAskUserTextWindow()
{
    ask_user_text_window_ = new AskUserTextWindow( NULL );
    ui.m_askUserTextWidget->layout()->addWidget( ask_user_text_window_ );
};
//---------------------------------------------------------------------------------
//
//
void RoseMain::initMessagesWindow()
{
    messages_window_ = new MessagesWindow( NULL );
    ui.m_messagesWidget->layout()->addWidget( messages_window_ );
}

void RoseMain::initManualPlatformControlWindow()
{
    ROS_INFO("RoseMain::initManualPlatformControlWindow()::begin");
    manual_platform_control_window_ = new ManualPlatformControlWindow( NULL );
    ROS_INFO("RoseMain::initManualPlatformControlWindow()::addwidget");
    ui.m_manual_platform_controlWidget->layout()->addWidget(manual_platform_control_window_);
    ROS_INFO("RoseMain::initManualPlatformControlWindow()::end");
}

void RoseMain::initJoystickModeSelection()
{
    ros::NodeHandle n;
    ui.m_JoystickModeSelector->setInsertPolicy(QComboBox::InsertAlphabetically);

    //! @todo LvB: Should be configurable and maybe even go via a backend node
    availableModesSubscriber = nodeHandle_.subscribe( "/joystick_teleop_interpreter/available_modes",  1, &RoseMain::CB_joystickmodesReceived, this);
    currentModeSubscriber = nodeHandle_.subscribe( "/joystick_teleop_interpreter/mode",  1, &RoseMain::CB_currentJoystickmodeReceived, this);
    ROS_DEBUG_NAMED(ROS_NAME, "Subscribed to /joystick_teleop_interpreter/available_modes");

    this->modeSelectionClient = nodeHandle_.serviceClient<rose_joystick::switch_joystick_mode>("/joystick_teleop_interpreter/set_mode");
}

//---------------------------------------------------------------------------------
//
//
void RoseMain::InitPublishers()
{
    goal_publisher          = n_.advertise<std_msgs::Int32>("/goal_sender_mode", 100);
    save_waypoint_pub_      = n_.advertise<std_msgs::Empty>("/map_display/save_current_location", 100);

    // move_forwards_pub_      = n_.advertise<std_msgs::Empty>("/manual_platform_control/move_forward",  1);
    // move_backwards_pub_     = n_.advertise<std_msgs::Empty>("/manual_platform_control/move_backward", 1);
    // strafe_left_pub_        = n_.advertise<std_msgs::Empty>("/manual_platform_control/strafe_left",   1);
    // stafe_right_pub_        = n_.advertise<std_msgs::Empty>("/manual_platform_control/stafe_right",   1);
    // rotate_left_pub_        = n_.advertise<std_msgs::Empty>("/manual_platform_control/rotate_left",   1);
    // rotate_right_pub_       = n_.advertise<std_msgs::Empty>("/manual_platform_control/rotate_right",  1);
    // stop_pub_               = n_.advertise<std_msgs::Empty>("/manual_platform_control/stop",          1);

    left_arm_select = n_.advertise<std_msgs::String>("/gui/left_arm_select", 100);    std_msgs::String msg;
    msg.data = "enable";
    left_arm_select.publish(msg);
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::DisplayTime()
{
    static time_t starttime;
    static time_t timestamp;
    time_t rawtime;
    struct tm * timeinfo;
    std::stringstream ss;

    time ( &rawtime );
    if ( starttime == 0 ) {
        timestamp = starttime = rawtime;
    }
    m_ElapsedTime = rawtime - starttime;

    timeinfo = localtime ( &rawtime );
    ss << asctime(timeinfo) << "  uptime: " << m_ElapsedTime;
    ui.m_labelDateTime->setText( ss.str().c_str() );
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::initPoseSubscription()
{
    //Connect the speed signal and slot
    //connect(this, SIGNAL(SpeedChanged(double)), this, SLOT(onSpeedChanged(double)));

    lastKnownPoseSet = false;
    poseSubscriber = nodeHandle_.subscribe("RosAria/pose", 10, &RoseMain::updatePoseCallback, this);
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::updatePoseCallback(nav_msgs::Odometry pose)
{
    //Change the current pose
    this->lastKnownPose = pose;
    lastKnownPoseSet = true;

    //Q_EMIT the speed changed signal
    //Q_EMIT SpeedChanged(tf::getYaw(pose.pose.pose.orientation));    //TODO: Check correct value taken
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::CreateApplicationServicesComponents()
{
    ROS_DEBUG("RoseMain::CreateApplicationServicesComponents::begin");
    m_BatteryController = boost::shared_ptr< BatteryController >( new BatteryController( m_Battery ) );
    m_LedController = boost::shared_ptr< LedController >( new LedController( m_HeartBeatLed ) );

    // Cameras
    m_HandCamController = boost::shared_ptr< CamController >( new CamController( m_cameraHand ) );
    m_OverviewCamController = boost::shared_ptr< CamController >( new CamController( m_cameraOverview ) );
    m_OperatorCamController = boost::shared_ptr< CamController >( new CamController( m_operatorCamera ) );
    ROS_DEBUG("RoseMain::CreateApplicationServicesComponents::end");
}

//---------------------------------------------------------------------------------
//
//
void RoseMain::CreateConnectionsToApplicationServices()
{
    ROS_DEBUG("RoseMain::CreateConnectionsToApplicationServices::begin");
    m_dependencies->BatteryMonitorListener->RegisterObserver( m_BatteryController );
    m_dependencies->BatteryMonitorListener->Subscribe();

    m_dependencies->CameraOverviewListener->RegisterObserver( m_OverviewCamController );
    m_dependencies->CameraOverviewListener->Subscribe();

    m_dependencies->CameraOperatorListener->RegisterObserver( m_OperatorCamController );
    m_dependencies->CameraOperatorListener->Subscribe();

    m_dependencies->HeartBeatListener->RegisterObserver( m_LedController );
    m_dependencies->HeartBeatListener->Subscribe();

    m_dependencies->PanMotorStateListener->RegisterObserver( m_panMotorStateListener );
    m_dependencies->PanMotorStateListener->Subscribe();

    m_dependencies->TiltMotorStateListener->RegisterObserver( m_tiltMotorStateListener );
    m_dependencies->TiltMotorStateListener->Subscribe();
    ROS_DEBUG("RoseMain::CreateConnectionsToApplicationServices::end");
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::on_m_ButtonHelp_clicked()
{
    new OperationSuccessful();
    // new ConnectionLost();
}

void RoseMain::CB_checkOperationSuccessful()
{
}

void RoseMain::CB_emergency( const bool& emergency )
{
    ROS_DEBUG_NAMED(ROS_NAME, "Emergency callback");
    if(emergency)
    {
        ui.m_ButtonStop->setText("Reset emergency");
        change_emergency_button_colow_timer_.start();
    }
    else
    {
        ui.m_ButtonStop->setText("Emergency stop");
        ui.m_ButtonStop->setStyleSheet("background-color: rgb(255, 0, 0); color: rgb(255, 255, 255)");
        change_emergency_button_colow_timer_.stop();
    }
}

void RoseMain::CB_changeStopButtonColor( const ros::TimerEvent& event )
{
    ROS_DEBUG_NAMED(ROS_NAME, "Change color");
    if ( stop_button_red_ )
    {
        ui.m_ButtonStop->setStyleSheet("background-color: rgb(255, 0, 0); color: rgb(255, 255, 255)");
    }
    else
    {
        ui.m_ButtonStop->setStyleSheet("background-color: rgb(255, 255, 255); color: rgb(255, 0, 0)");
    }

    stop_button_red_ = !stop_button_red_;
}

//---------------------------------------------------------------------------------
//
//
void RoseMain::StartVoicePlayback()
{
    m_alsaPlayback = Alsa::AlsaPlayback::Ptr(new Alsa::AlsaPlayback(m_dependencies->VoiceListener));
    m_alsaPlayback->setBufferingDepth(3);
    m_dependencies->VoiceListener->RegisterObserver(m_alsaPlayback) ;
    m_playbackThread = ApplicationServices::ThreadRunner::Ptr(new ApplicationServices::ThreadRunner(m_alsaPlayback, "Alsa Playback")) ;
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::StartOperatorCam()
{
    ROS_DEBUG("RoseMain::StartOperatorCam::begin");
    std::string OperatorCamDevice = m_dependencies->CockpitConfiguration->GetOperatorCamDevice();
    std::string OperatorCamTopic = m_dependencies->CockpitConfiguration->GetOperatorCamTopic();
    Ros::RosWrapper::Ptr ros_wrapper =  m_dependencies->RosWrapper;

    m_OperatorCamCapture = Video::VideoCapture::Ptr( new Video::VideoCapture( OperatorCamDevice, OperatorCamTopic, ros_wrapper ) );
    m_OperatorCamCaptureThread = ApplicationServices::ThreadRunner::Ptr( 
        new ApplicationServices::ThreadRunner( m_OperatorCamCapture, "operatorCamCapture" ));

    this->ui.m_showOperatorCamCheckBox->setChecked(m_OperatorCamCapture->isStarted());
    m_operatorCamera->setHidden(this->ui.m_showOperatorCamCheckBox->checkState() != Qt::Checked);

    ROS_DEBUG("RoseMain::StartOperatorCam::end");
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::CreatePanAndTiltEndListeners()
{
    m_panMotorStateListener = ApplicationServices::MotorStateListener::Ptr(
            new ApplicationServices::MotorStateListener()
        );

    m_tiltMotorStateListener = ApplicationServices::MotorStateListener::Ptr(
            new ApplicationServices::MotorStateListener()
        );
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::CreateOperatorCamViewer()
{
    ROS_DEBUG("RoseMain::CreateOperatorCamViewer::begin");
    m_operatorCamera = new CameraDisplay(0, m_cameraOverview);

    int height = m_cameraOverview->height()/4;
    int width = m_cameraOverview->width()/4;

    // Move to lower right bottom
    m_operatorCamera->setSceneRect(0, 0, width, height);
    m_operatorCamera->move(m_cameraOverview->width() - width, m_cameraOverview->height() - height);
    connect(m_operatorCamera, SIGNAL(ImageChanged()), this, SLOT(on_m_operatorCameraImageChanged()));
    ROS_DEBUG("RoseMain::CreateOperatorCamViewer::end");
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::on_m_operatorCameraImageChanged()
{
    ROS_DEBUG("RoseMain::on_m_operatorCameraUpdate::begin");
    ROS_DEBUG("RoseMain::on_m_operatorCameraUpdate::end");
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::CreateOverviewCamViewer()
{
    ROS_DEBUG("RoseMain::CreateOverviewCamViewer::begin");
    unsigned int components = InteractiveImageDisplay::ocDefaultPosControl |
                              InteractiveImageDisplay::ocPanTilt |
                              InteractiveImageDisplay::ocCenterDisplay;
    //
    m_cameraOverview = new OverviewCameraDisplay( components, NULL );
    ui.m_cameraOverView->layout()->addWidget( m_cameraOverview );

    CreateOverviewCamInteractionObject();
    m_cameraOverview->SetInteractionObject( m_OverviewInteractionObject );
    //m_cameraOverview->setSceneRect(0,0,320,240);
    m_cameraOverview->setEnabled( true );
    ROS_DEBUG("RoseMain::CreateOverviewCamViewer::end");
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::CreateOverviewCamInteractionObject()
{
    CreatePanAndTiltEndListeners();
    OverviewCameraInteractionObject<Ros::RosWrapper>::OverviewCameraInteractionObjectDependencies dependencies;
    dependencies.m_PanMotorStateListener = m_panMotorStateListener;
    dependencies.m_TiltMotorStateListener = m_tiltMotorStateListener;
    dependencies.m_RosWrapper = m_dependencies->RosWrapper;
    dependencies.m_CockpitConfiguration = m_dependencies->CockpitConfiguration;
    //
    m_OverviewInteractionObject = boost::shared_ptr<InteractionObject>( new OverviewCameraInteractionObject<Ros::RosWrapper>( dependencies ) );
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::CreateHandCamViewer()
{
    ROS_DEBUG("RoseMain::CreateHandCamViewer::begin");
    unsigned int components =  0; /*InteractiveImageDisplay::ocDefaultPosControl |
                               InteractiveImageDisplay::ocPanTilt;
                               | InteractiveImageDisplay::ocCamSelectors;*/

    m_cameraHand = new CameraDisplay( components, NULL );
    ui.m_cameraHand->layout()->addWidget(m_cameraHand);

//    CreateHandCamInteractionObject();
//    m_cameraHand->SetInteractionObject( m_HandInteractionObject );
//    m_cameraHand->SelectCam( current_hand_camera_ );
//    m_cameraHand->setEnabled( true );

    ROS_DEBUG("RoseMain::CreateHandCamViewer::end");
    
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::CreateHandCamInteractionObject()
{
    HandCameraInteractionObject<Ros::RosWrapper>::HandCameraInteractionObjectDependencies dependencies;
    //
    dependencies.m_MainForm = this;//RoseMain::Ptr( this );
    dependencies.m_HandCamController = m_HandCamController;
    dependencies.m_CameraLeftHandListener = m_dependencies->CameraLeftHandListener;
    dependencies.m_CameraRightHandListener = m_dependencies->CameraRightHandListener;
    dependencies.m_RosWrapper = m_dependencies->RosWrapper;
    dependencies.m_CockpitConfiguration = m_dependencies->CockpitConfiguration;
    //
    m_HandInteractionObject = boost::shared_ptr<InteractionObject>( new HandCameraInteractionObject<Ros::RosWrapper>( dependencies ) );
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::CreateMapDisplayViewer()
{
    ROS_DEBUG("RoseMain::CreateMapDisplayViewer::begin");
    unsigned int components =  InteractiveImageDisplay::ocDefaultPosControl |
                               InteractiveImageDisplay::ocPanTilt |
                               InteractiveImageDisplay::ocWaypointsOverlay |
                               InteractiveImageDisplay::ocZoomBar;

    m_MapDisplay = new MapDisplay( components, NULL );
    ui.m_HouseLayoutView->layout()->addWidget( m_MapDisplay );

    CreateMapDisplayInteractionObject();
    m_MapDisplay->SetInteractionObject( m_MapDisplayInteractionObject );
    m_MapDisplay->setEnabled( true );

    connect(m_MapDisplay, SIGNAL(setNavGoalStarted()),              this, SLOT(on_navigationGoalSelection_selected()));
    connect(m_MapDisplay, SIGNAL(setPose2dStarted()),               this, SLOT(on_PoseSelection_selected()));
    connect(m_MapDisplay, SIGNAL(setPoseOrNavSelectionStopped()),   this, SLOT(on_noNavigationOrPoseSelection_selected()));
    connect(m_MapDisplay, SIGNAL(setNavGoalDone()),                 this, SLOT(on_noNavigationCheckbox_clicked()));
    connect(m_MapDisplay, SIGNAL(setPose2dDone()),                  this, SLOT(on_noNavigationCheckbox_clicked()));

    ROS_DEBUG("RoseMain::CreateMapDisplayViewer::end");
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::CreateMapDisplayInteractionObject()
{
    MapDisplayInteractionObject<Ros::RosWrapper>::MapDisplayInteractionObjectDependencies dependencies;
  
    dependencies.m_CockpitConfiguration = m_dependencies->CockpitConfiguration;
    dependencies.m_RosWrapper = m_dependencies->RosWrapper;

    m_MapDisplayInteractionObject = boost::shared_ptr<InteractionObject>( new MapDisplayInteractionObject<Ros::RosWrapper>( dependencies ) );
}

// @todo: HACK
void RoseMain::on_m_ButtonStop_clicked()
{
    ROS_INFO("RoseMain::on_m_ButtonStop_clicked");   
    if(sh_emergency_)
        sh_emergency_ = false;
    else
        sh_emergency_ = true;
    
    CB_emergency( sh_emergency_ );
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::InitGui()
{
    ROS_DEBUG("RoseMain::InitGui::begin");

    ui.m_labelRoseNavigator->setText( "<font color=\"#008ED3\">ROSE Navigator</font>" );

    ui.m_labelRoseLogoSmallTopLeft->setPixmap( QPixmap(":resources/rose.png" ) );
    ui.m_labelRoseLogoSmallTopLeft->setScaledContents( true );
    //
    LoadImagesForHandControl();
    InitializeButtons();

    AddAllSliders();
    AddBattery();
    AddHeartBeatLed();
    CreateOverviewCamViewer();
    CreateOperatorCamViewer();
    CreateHandCamViewer();
    CreateMapDisplayViewer();
    Configure1ArmAutonomousButtons();
    Configure2ArmAutonomousButtons();

    ROS_DEBUG("RoseMain::InitGui::end");
}

//---------------------------------------------------------------------------------
//
//
void RoseMain::InitializeButtons()
{
    ui.m_ButtonClose->setStyleSheet( TsrDefinitions::Instance()->GetRedButtonStyleSheet().c_str() );
    ui.m_ButtonClose->setIcon(QIcon(":/resources/pwr.png"));
    ui.m_ButtonClose->setText("");
    ui.m_ButtonClose->setIconSize(QSize(20, 20));
    //
    ui.m_ButtonPause->setStyleSheet( TsrDefinitions::Instance()->GetDefaultButtonStyleSheet().c_str() );
    //
    ui.m_ButtonHelp->setStyleSheet( TsrDefinitions::Instance()->GetDefaultButtonStyleSheet().c_str() );
    //
    ui.m_ButtonStop->setStyleSheet( TsrDefinitions::Instance()->GetRedButtonStyleSheet().c_str() );
    this->telepresenceIsOn  = false;

    this->operatorVoiceIsOn = true;
    this->customerVoiceIsOn = true;

    
    // Navigation buttons
    // ui.moveBackwardButton->setIcon(QIcon(":/resources/downArrow.png"));
    // ui.moveBackwardButton->setText("");

    // ui.moveForwardButton->setIcon(QIcon(":/resources/upArrow.png"));
    // ui.moveForwardButton->setText("");

    // ui.strafeLeftButton->setIcon(QIcon(":/resources/leftArrow.png"));
    // ui.strafeLeftButton->setText("");

    // ui.strafeRightButton->setIcon(QIcon(":/resources/rightArrow.png"));
    // ui.strafeRightButton->setText("");

    // ui.rotateLeftButton->setIcon(QIcon(":/resources/rotateLeft.png"));
    // ui.rotateLeftButton->setText("");

    // ui.rotateRightButton->setIcon(QIcon(":/resources/rotateRight.png"));
    // ui.rotateRightButton->setText("");
    
    // ui.stopMovementButton->setIcon(QIcon(":/resources/stopMovement.png"));
    // ui.stopMovementButton->setText("");

    // Map checkboxes
    ui.noNavigationCheckbox->setChecked(true);
    ui.navigationCheckbox->setChecked(false);
    ui.posEstimateCheckbox->setChecked(false);
}

void RoseMain::on_m_saveWaypointButton_clicked()
{
    ROS_DEBUG("on_m_saveWaypointButton_clicked");

    std_msgs::Empty empty;
    save_waypoint_pub_.publish(empty);
}

void RoseMain::on_noNavigationCheckbox_clicked()
{
    m_MapDisplay->setNoNavigation();
}

void RoseMain::on_navigationCheckbox_clicked()
{
    m_MapDisplay->setNavigation();
}

void RoseMain::on_posEstimateCheckbox_clicked()
{
    m_MapDisplay->set2DPositioning();
}

void RoseMain::on_noNavigationOrPoseSelection_selected()
{
    ui.noNavigationCheckbox->setChecked(true);
    ui.navigationCheckbox->setChecked(false);
    ui.posEstimateCheckbox->setChecked(false);
}

void RoseMain::on_navigationGoalSelection_selected()
{
     ui.noNavigationCheckbox->setChecked(false);
     ui.navigationCheckbox->setChecked(true);
     ui.posEstimateCheckbox->setChecked(false);
}

void RoseMain::on_PoseSelection_selected()
{
    ui.noNavigationCheckbox->setChecked(false);
    ui.navigationCheckbox->setChecked(false);
    ui.posEstimateCheckbox->setChecked(true);
}

//
//---------------------------------------------------------------------------------------------------------------------
//
void RoseMain::AddHeartBeatLed()
{
    m_HeartBeatLed = boost::shared_ptr< CLed >( new CLed( ui.m_GeneralControlsPage ) );
    m_HeartBeatLed->resize( 20, 20 );
    m_HeartBeatLed->move( 150, 53 );
    m_HeartBeatLed->show();
}
//
//---------------------------------------------------------------------------------------------------------------------
//
void RoseMain::AddBattery()
{
    m_Battery = boost::shared_ptr< Battery >( new Battery( ui.m_GeneralControlsPage ) );
    m_Battery->resize( 220, 115 );
    m_Battery->move( 300, 5 );
    m_Battery->show();
}

//---------------------------------------------------------------------------------------------------------------------
//
//
void RoseMain::Configure1ArmAutonomousButtons()
{
    
}
//---------------------------------------------------------------------------------------------------------------------
//
//
void RoseMain::Configure2ArmAutonomousButtons()
{
    
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::closeEvent ( QCloseEvent * event )
{
    std::cout << "Closing RoseMain..." << std::endl;
    
    m_dependencies->HeartBeatListener->Unsubscribe();
    m_dependencies->CameraOverviewListener->Unsubscribe();
    m_dependencies->CameraLeftHandListener->Unsubscribe();
    m_dependencies->CameraRightHandListener->Unsubscribe();
    m_dependencies->CameraOperatorListener->Unsubscribe();
    m_dependencies->VoiceListener->Unsubscribe();
    m_dependencies->PanMotorStateListener->Unsubscribe();
    m_dependencies->PanMotorStateListener->Unsubscribe();
    m_dependencies->BatteryMonitorListener->Unsubscribe();
    event->accept();
}
//---------------------------------------------------------------------------------
//
//
#define VAL_SIZE 40
bool RoseMain::event ( QEvent * pevent )
{
    //     2011/04/14    nv ToDo debug activity: sort me out of clean me up
    //     static unsigned int cnt;
    //     static unsigned int val[VAL_SIZE];
    //     unsigned int i = cnt++ % VAL_SIZE;
    //
    //     if ( i == 0 ) {
    //         std::cout << "RoseMain::event (" << cnt << ")  ";
    //         for ( int j = 0; j < VAL_SIZE; j++ ) {
    //             std::cout << val[j] << " ";
    //         }
    //         std:: cout << std::endl;
    //     }
    //     val[i] = pevent->type();

    if ( pevent->type() == MAINQUIT_EVENT )
    {
        std::cout << "MAINQUIT_EVENT" << std::endl;
        //on_m_buttonCall_clicked();

        // don't care about return value;
        QGenericReturnArgument returnvalue;
        //QMetaObject* metaObject = (QMetaObject*)m_roseStartup->ui.m_buttonCall->metaObject();
        QMetaObject* metaObject = (QMetaObject*)this->metaObject();
        bool result = metaObject->invokeMethod( this, "on_m_ButtonClose_clicked", Qt::DirectConnection, returnvalue );
        if( !result )
            std::cout << "Test is failing, since the signal could not be invoked ( signal name = " << "on_m_ButtonClose_clicked" << " )" << std::endl;
        return true;
    }
    return QWidget::event(pevent);
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::on_m_ButtonClose_clicked()
{
    close();
}

//---------------------------------------------------------------------------------
//
//
void RoseMain::on_m_buttonOperatorVolumeMute_clicked(bool state)
{
    this->operatorVoiceClient = nodeHandle_.serviceClient<roscomm::toggle_operator_voice_capturer>("/toggle_OperatorVoice");
    roscomm::toggle_operator_voice_capturer srv;

    srv.request.toggle_operator_voice_capturer_request = !this->operatorVoiceIsOn;

    if(this->operatorVoiceClient.call(srv))
    {
        this->operatorVoiceIsOn = srv.request.toggle_operator_voice_capturer_request;

        ROS_INFO("Operator voice status: %d", srv.response.toggle_operator_voice_capturer_response);

        if(this->operatorVoiceIsOn)
        {
            this->ui.m_buttonOperatorVolumeMute->setIcon(QIcon(":/resources/microphone.png"));
        }
        else
        {
            this->ui.m_buttonOperatorVolumeMute->setIcon(QIcon(":/resources/microphone--strikethrough.png"));
        }
    }
    else
    {
        ROS_ERROR("Failed to call service /toggle_OperatorVoice");
    }

    (*(ui.m_buttonOperatorVolumeMute)).show();

}

//---------------------------------------------------------------------------------
//
//
void RoseMain::on_m_buttonClientVolumeMute_clicked(bool state)
{
    ROS_DEBUG("on_m_buttonClientVolumeMute_clicked");

    ros::NodeHandle n;
    this->customerVoiceClient = n.serviceClient<roscomm::toggle_customer_voice_capturer>("/toggle_CustomerVoice");
    roscomm::toggle_customer_voice_capturer srv;

    this->customerVoiceIsOn = state;
    srv.request.toggle_customer_voice_capturer_request = this->customerVoiceIsOn;

    if(this->customerVoiceClient.call(srv))
    {
        ROS_INFO("Customer voice status: %d", srv.response.toggle_customer_voice_capturer_response);

        this->customerVoiceIsOn = srv.request.toggle_customer_voice_capturer_request;

        if(this->customerVoiceIsOn)
        {
            this->ui.m_buttonClientVolumeMute->setIcon(QIcon(":/resources/microphone.png"));
        }
        else
        {
            this->ui.m_buttonClientVolumeMute->setIcon(QIcon(":/resources/microphone--strikethrough.png"));
        }
    }
    else
    {
        ROS_ERROR("Failed to call service /toggle_CustomerVoice");
    }

    (*(ui.m_buttonClientVolumeMute)).show();

}

void RoseMain::on_m_buttonOperatorVideoMute_clicked(bool state)
{
    ROS_DEBUG("on_m_buttonOperatorVideoMute_clicked");

    if(!state)
    {
        ROS_DEBUG("on_m_buttonOperatorVideoMute_clicked: pauzing operator feed");
        m_OperatorCamCapture->pause();

        this->ui.m_buttonOperatorVideoMute->setIcon(QIcon(":/resources/webcam--strikethrough.png"));
    }
    else
    {
        ROS_DEBUG("on_m_buttonOperatorVideoMute_clicked: UNpauzing operator feed");
        m_OperatorCamCapture->unpause();
        this->ui.m_buttonOperatorVideoMute->setIcon(QIcon(":/resources/webcam.png"));
    }
    (*(ui.m_buttonOperatorVideoMute)).show();
}

void RoseMain::on_m_buttonClientVideoMute_clicked(bool state)
{
    ROS_DEBUG("on_m_buttonClientVideoMute_clicked");

    if(state)
    {
        this->ui.m_buttonClientVideoMute->setIcon(QIcon(":/resources/webcam.png"));
    }
    else
    {
        this->ui.m_buttonClientVideoMute->setIcon(QIcon(":/resources/webcam--strikethrough.png"));
    }
}

//--------------------------------------------------------------------------------
//
//
void RoseMain::on_m_showOperatorCamCheckBox_clicked()
{
    m_operatorCamera->setHidden(this->ui.m_showOperatorCamCheckBox->checkState() != Qt::Checked);
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::AddAllSliders()
{
    AddOperatorVoiceSlider();
    AddClientVoiceSlider();
    //
    AddHandOpenCloseSlider();
    AddHandPressureSlider();
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::AddOperatorVoiceSlider()
{
    m_OperatorVoiceSliderSlave = SliderSlave::Ptr( new EmptySliderSlave("Operator volume") );

    m_OperatorVoiceSlider = new TSRSlider( NULL, m_OperatorVoiceSliderSlave );
    m_OperatorVoiceSlider->move( 150, 194 );
    m_OperatorVoiceSlider->resize( 201, 30 );
    m_OperatorVoiceSlider->SetSliderMode( Slider::smDirectFollow );
    m_OperatorVoiceSlider->SetTickEnable( false );

    ui.m_OperatorVoiceSliderLayout->addWidget( m_OperatorVoiceSlider );
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::AddClientVoiceSlider()
{
    m_ClientVoiceSliderSlave = SliderSlave::Ptr( new EmptySliderSlave("Client volume") );

    m_ClientVoiceSlider = new TSRSlider( ui.m_tabCommunication, m_ClientVoiceSliderSlave );
    m_ClientVoiceSlider->move( 150, 234 );
    m_ClientVoiceSlider->resize( 181, 30 );
    m_ClientVoiceSlider->SetSliderMode( Slider::smDirectFollow );
    m_ClientVoiceSlider->SetTickEnable( false );

    ui.m_ClientVoiceSliderLayout->addWidget( m_ClientVoiceSlider );
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::AddHandOpenCloseSlider()
{
    m_HandOpenCloseSliderSlave = SliderSlave::Ptr( new PublishingSliderSlave("Hand open close", "/gui/gripper_width") );

    m_HandOpenCloseSlider = new TSRSlider( NULL, m_HandOpenCloseSliderSlave );
    m_HandOpenCloseSlider->move( 60, 40 );
    m_HandOpenCloseSlider->resize( 300, 30 );
    m_HandOpenCloseSlider->SetSliderMode( Slider::smDirectFollow );
    m_HandOpenCloseSlider->SetTickEnable( true );

    ui.m_OpenCloseSliderLayout->addWidget( m_HandOpenCloseSlider );
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::AddHandPressureSlider()
{
    m_HandPressureSliderSlave = SliderSlave::Ptr( new PublishingSliderSlave("Hand pressure", "/gui/gripper_pressure") );

    m_HandPressureSlider = new TSRSlider( NULL, m_HandPressureSliderSlave );
    m_HandPressureSlider->move( 60, 60 );
    m_HandPressureSlider->resize( 300, 30 );
    m_HandPressureSlider->SetSliderMode( Slider::smDirectFollow );
    m_HandPressureSlider->SetTickEnable( true );

    ui.m_PressureSliderLayout->addWidget( m_HandPressureSlider );
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::LoadImagesForHandControl()
{
    ui.m_CupLabel->setPixmap( QPixmap(":resources/32x32_ico_mok.png" ) );
    ui.m_EggLabel->setPixmap( QPixmap(":resources/32x32_ico_egg.png" ) );
    ui.m_WeightLabel->setPixmap( QPixmap(":resources/32x32_ico_weight.png" ) );
    //
    //    ui.m_OpenGripperLabel->setPixmap( QPixmap(":resources/gripperopen.png" ) );
    //    ui.m_ClosedGripperLabel->setPixmap( QPixmap(":resources/gripperclosed.png" ) );
    ui.m_PressureGripperLabel->setPixmap( QPixmap(":resources/gripperpressure.png" ) );
}
//---------------------------------------------------------------------------------
//
//! @todo MdL: Move manual movement to separate block
void RoseMain::on_moveForwardButton_clicked()
{
    std_msgs::Empty empty;
    move_forwards_pub_.publish(empty);
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::on_moveBackwardButton_clicked()
{
    std_msgs::Empty empty;
    move_backwards_pub_.publish(empty);
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::on_strafeLeftButton_clicked()
{
    std_msgs::Empty empty;
    strafe_left_pub_.publish(empty);
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::on_strafeRightButton_clicked()
{
    std_msgs::Empty empty;
    stafe_right_pub_.publish(empty);
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::on_rotateLeftButton_clicked()
{
    std_msgs::Empty empty;
    rotate_left_pub_.publish(empty);
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::on_rotateRightButton_clicked()
{
    std_msgs::Empty empty;
    rotate_right_pub_.publish(empty);
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::on_stopMovementButton_clicked()
{
    std_msgs::Empty empty;
    stop_pub_.publish(empty);
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::on_liftSlider_sliderPressed()
{
    // updateLiftSlider();
}

//---------------------------------------------------------------------------------
//
//
void RoseMain::on_liftSlider_sliderReleased()
{
    // updateLiftSlider();
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::on_liftSlider_valueChanged()
{
    // updateLiftSlider();
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::updateLiftSlider()
{   
    // roscomm::lift lift_message;
    // lift_message.enabled = true;
    // lift_message.pose = ui.liftSlider->value();

    // lift_publisher.publish(lift_message);
    // ROS_INFO_NAMED(ROS_NAME, "RoseMain::on_liftSlider_sliderReleased: %i", ui.liftSlider->value());
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::on_m_ClosedGripperButton_clicked()
{
    std_msgs::String msg;
    msg.data = "enable";
    left_arm_select.publish(msg);

    m_HandOpenCloseSlider->SetCurrentValue(0);

    m_HandOpenCloseSliderSlave->NewSetpointAvailable(0);
    m_HandOpenCloseSliderSlave->ChangeFinished();
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::on_m_OpenedGripperButton_clicked()
{
    std_msgs::String msg;
    msg.data = "enable";
    left_arm_select.publish(msg);

    m_HandOpenCloseSlider->SetCurrentValue(100);

    m_HandOpenCloseSliderSlave->NewSetpointAvailable(100);
    m_HandOpenCloseSliderSlave->ChangeFinished();
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::on_m_JoystickModeSelector_currentIndexChanged(QString text)
{
    ROS_DEBUG_NAMED(ROS_NAME, "RoseMain::on_m_JoystickModeSelector_currentIndexChanged(%s)", text.toStdString().c_str());

    rose_joystick::switch_joystick_mode modeSwitch;
    modeSwitch.request.switch_to_mode = text.toStdString();
    this->modeSelectionClient.call(modeSwitch);
}
//---------------------------------------------------------------------------------
//
//
void RoseMain::CB_joystickmodesReceived(const roscomm::stringlist &modes)
{
//    ROS_DEBUG_NAMED(ROS_NAME, "RoseMain::CB_joystickmodesReceived received %i available modes", (int)modes.values.size());
    for(const std::string mode : modes.values)
    {
        QString modeString = QString(mode.c_str());
        if(ui.m_JoystickModeSelector->findText(modeString) == -1) //Item not in list already
        {
//            ROS_DEBUG_NAMED(ROS_NAME, "Adding %s", mode.c_str());
            ui.m_JoystickModeSelector->addItem(QString(modeString));
        }
        else
        {
//            ROS_DEBUG_NAMED(ROS_NAME, "%s already exists", mode.c_str());
        }
    }

    std::vector<int> to_remove;
    for(int i=0; i<ui.m_JoystickModeSelector->count()-1; i++)
    {
        QString txt = ui.m_JoystickModeSelector->itemText(i);
        std::string std_txt = txt.toStdString();

//        ROS_DEBUG_NAMED(ROS_NAME, "Checking whether '%s'@%i is still an available mode", std_txt.c_str(), i);

        if(!std_txt.empty() && std::find(modes.values.begin(), modes.values.end(), std_txt) == modes.values.end())
        {
            ROS_DEBUG_NAMED(ROS_NAME, "'%s'@%i is no longer an available mode", std_txt.c_str(), i);
            to_remove.push_back(i);
        }
    }

    ROS_DEBUG_NAMED(ROS_NAME, "There are %i items to remove", (int)to_remove.size());
    for(const int index : to_remove)
    {
//       ROS_DEBUG_NAMED(ROS_NAME, "removeItem(%i = %s)", index, ui.m_JoystickModeSelector->itemText(index).toStdString().c_str());
       ui.m_JoystickModeSelector->removeItem(index);
    }
}

void RoseMain::CB_currentJoystickmodeReceived(const std_msgs::String &msg)
{
    //ROS_DEBUG_NAMED(ROS_NAME, "RoseMain::CB_currentJoystickmodeReceived current mode: %s", msg.data.c_str());

    QString modeString = QString(msg.data.c_str());

    bool oldState = ui.m_JoystickModeSelector->blockSignals(true); //Rember whether signals are enabled or not and disable them for now

    int index = ui.m_JoystickModeSelector->findText(modeString);
    //With signals disabled, change the selected item. Because the signals are disabled, this will not trigger a signal. We don't want this now, because its not user triggered.
    ui.m_JoystickModeSelector->setCurrentIndex(index);

    ui.m_JoystickModeSelector->blockSignals(oldState); //Unblock signals again: restore the original state.
}
