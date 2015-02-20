/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/04/15
* 		- File created.
*
* Description:
*	This code emulates a joystick, which is interpreted by a joystick node to control the robot.
* 
***********************************************************************************/
#include "rose_main/manual_platform_control_window/manual_platform_control_window.hpp"

ManualPlatformControlWindow::ManualPlatformControlWindow( QWidget* parent )
    : parent_ ( parent )
{   
    ROS_INFO("ManualPlatformControlWindow::ManualPlatformControlWindow");

    joystick_pub_       = n_.advertise<sensor_msgs::Joy>("/manual_platform_control/joy",  1);

    lift_publisher_     = n_.advertise<rose_base_msgs::lift_command>("lift_controller/lift/command", 1);

    // Initialize all axes to 0
    axes_.resize(3);

    createLayout();

    publish_timer_      = n_.createTimer(ros::Duration(0.05), &ManualPlatformControlWindow::publishJoystickMessage, this, false, false);

    ROS_INFO("ManualPlatformControlWindow::ManualPlatformControlWindow::end");
}

ManualPlatformControlWindow::~ManualPlatformControlWindow()
{
}

void ManualPlatformControlWindow::createLayout()
{
    ROS_INFO("ManualPlatformControlWindow::createLayout()");
    QGridLayout* layout = new QGridLayout;

    ROS_INFO("ManualPlatformControlWindow::new layout created");
    // Buttons for navigation
    move_forward_button_ = new QPushButton();
    move_forward_button_->setIcon(QIcon(":/resources/upArrow.png"));
    move_forward_button_->setText("");
    connect(move_forward_button_, SIGNAL(clicked()), this, SLOT(CB_move_forward_button_clicked()));
    layout->addWidget(move_forward_button_,0,1);

    move_backward_button_ = new QPushButton();
    move_backward_button_->setIcon(QIcon(":/resources/downArrow.png"));
    move_backward_button_->setText("");
    connect(move_backward_button_, SIGNAL(clicked()), this, SLOT(CB_move_backward_button_clicked()));
    layout->addWidget(move_backward_button_,2,1);

    strafe_left_button_ = new QPushButton();
    strafe_left_button_->setIcon(QIcon(":/resources/leftArrow.png"));
    strafe_left_button_->setText("");
    connect(strafe_left_button_, SIGNAL(clicked()), this, SLOT(CB_strafe_left_button_clicked()));
    layout->addWidget(strafe_left_button_,1,0);

    strafe_right_button_ = new QPushButton();
    strafe_right_button_->setIcon(QIcon(":/resources/rightArrow.png"));
    strafe_right_button_->setText("");
    connect(strafe_right_button_, SIGNAL(clicked()), this, SLOT(CB_strafe_right_button_clicked()));
    layout->addWidget(strafe_right_button_,1,2);

    rotate_left_button_ = new QPushButton();
    rotate_left_button_->setIcon(QIcon(":/resources/rotateLeft.png"));
    rotate_left_button_->setText("");
    connect(rotate_left_button_, SIGNAL(clicked()), this, SLOT(CB_rotate_left_button_clicked()));
    layout->addWidget(rotate_left_button_,0,0);

    rotate_right_button_ = new QPushButton();
    rotate_right_button_->setIcon(QIcon(":/resources/rotateRight.png"));
    rotate_right_button_->setText("");
    connect(rotate_right_button_, SIGNAL(clicked()), this, SLOT(CB_rotate_right_button_clicked()));
    layout->addWidget(rotate_right_button_,0,2);

    stop_movement_button_ = new QPushButton();
    stop_movement_button_->setIcon(QIcon(":/resources/stopMovement.png"));
    stop_movement_button_->setText("");
    connect(stop_movement_button_, SIGNAL(clicked()), this, SLOT(CB_stop_movement_button_clicked()));
    layout->addWidget(stop_movement_button_,1,1);

    // Lift control
    //! @todo MdL: lift control buttons
    lift_slider_ = new QSlider();
    lift_slider_->setMaximum(100);
    lift_slider_->setSingleStep(1);
    lift_slider_->setPageStep(1);
    //! @todo Mdl: Set the current position of the lift
    connect(lift_slider_, SIGNAL(valueChanged(int)), this, SLOT(CB_slider_value_changed(int)));

    layout->addWidget(lift_slider_,0,3,3,1);

    QLabel* up_label    = new QLabel("Up");
    QLabel* mid_label   = new QLabel("Middle");
    QLabel* down_label  = new QLabel("Down");

    layout->addWidget(up_label,0,4);
    layout->addWidget(mid_label,1,4);
    layout->addWidget(down_label,2,4);

    ROS_INFO("ManualPlatformControlWindow::setLayout()");
    setLayout(layout);
}

void ManualPlatformControlWindow::CB_move_forward_button_clicked()
{
    increase(axes_[0]);
    publish_timer_.start();
}

void ManualPlatformControlWindow::CB_move_backward_button_clicked()
{
    decrease(axes_[0]);
    publish_timer_.start();
}

void ManualPlatformControlWindow::CB_strafe_left_button_clicked()
{
    increase(axes_[1]);
    publish_timer_.start();
}

void ManualPlatformControlWindow::CB_strafe_right_button_clicked()
{
    decrease(axes_[1]);
    publish_timer_.start();
}

void ManualPlatformControlWindow::CB_rotate_right_button_clicked()
{
    increase(axes_[2]);
    publish_timer_.start();
}

void ManualPlatformControlWindow::CB_rotate_left_button_clicked()
{
    decrease(axes_[2]);
    publish_timer_.start();
}

void ManualPlatformControlWindow::CB_stop_movement_button_clicked()
{
    stopMovement();
    publish_timer_.start();
}

void ManualPlatformControlWindow::stopMovement()
{
    for ( auto& axis : axes_ )
        axis = 0.0;
}

void ManualPlatformControlWindow::publishJoystickMessage( const ros::TimerEvent& event )
{
    // If all axis are near 0.0, set all axis to 0.0
    float sum_of_all_axis = 0.0;
    for ( auto& axis : axes_ )
        sum_of_all_axis += std::abs(axis);

    bool movement_stopped = (sum_of_all_axis < getStepSize());
    if (movement_stopped)
        stopMovement();

    sensor_msgs::Joy joy;
    joy.axes = axes_;
    joystick_pub_.publish(joy);

    if (movement_stopped)
        publish_timer_.stop();
}

void ManualPlatformControlWindow::increase( float& value )
{
    if ( value + getStepSize() < JOY_MAX )
        value += getStepSize();
}

void ManualPlatformControlWindow::decrease( float& value )
{
     if ( value - getStepSize() > JOY_MIN )
        value -= getStepSize();
}

float ManualPlatformControlWindow::getStepSize()
{
    float step_size = 1/CLICKS_TO_MAX;
    return step_size;
}

void ManualPlatformControlWindow::CB_slider_value_changed( int value )
{
    rose_base_msgs::lift_command lift_msg;
    lift_msg.speed_percentage       = 100.0;
    lift_msg.position_percentage    = (float)value;

    lift_publisher_.publish(lift_msg);
    ROS_INFO("ManualPlatformControlWindow::on_liftSlider_sliderReleased: %d", value);
}
