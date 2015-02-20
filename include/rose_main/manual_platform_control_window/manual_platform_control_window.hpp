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
#ifndef MANUAL_PLATFORM_CONTROL_WINDOW_HPP
#define MANUAL_PLATFORM_CONTROL_WINDOW_HPP

#include <ros/ros.h>
#include <QtGui>
#include <QtGui/QWidget>
#include <QGridLayout>
#include <QLabel>
#include <stdio.h>
#include <stdlib.h>

#include "rose_base_msgs/lift_state.h"
#include "rose_base_msgs/lift_command.h"
// #include "std_msgs/Empty.h"
#include "sensor_msgs/Joy.h"

#define CLICKS_TO_MAX 10.0
#define NR_AXES 3
#define JOY_MIN -1.0
#define JOY_MAX 1.0

class ManualPlatformControlWindow : public QWidget
{
	Q_OBJECT

  public:
	ManualPlatformControlWindow( QWidget* parent );
	~ManualPlatformControlWindow();

  private:  
	void 	createLayout();
	void 	stopMovement();
	void 	publishJoystickMessage( const ros::TimerEvent& event );
	void 	increase( float& value );
	void 	decrease( float& value );
	float 	getStepSize();

    ros::NodeHandle  			n_;

    QPushButton*				move_forward_button_;
    QPushButton*				move_backward_button_;
    QPushButton*				strafe_left_button_;
    QPushButton*				strafe_right_button_;
    QPushButton*				rotate_left_button_;
    QPushButton*				rotate_right_button_;    
    QPushButton*				stop_movement_button_;
    QSlider*					lift_slider_;

    QWidget* 					parent_;

	ros::Publisher 				joystick_pub_;
	ros::Publisher 				lift_publisher_;

    std::vector<float>			axes_;
    ros::Timer 					publish_timer_;

  private Q_SLOTS:
	void CB_move_forward_button_clicked();
	void CB_move_backward_button_clicked();
	void CB_strafe_left_button_clicked();
	void CB_strafe_right_button_clicked();
	void CB_rotate_left_button_clicked();
	void CB_rotate_right_button_clicked();
	void CB_stop_movement_button_clicked();

	void CB_slider_value_changed( int value );
};

#endif // MANUAL_PLATFORM_CONTROL_WINDOW_HPP
