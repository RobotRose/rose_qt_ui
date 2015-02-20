/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/02/26
* 		- File created.
*
* Description:
*	description
* 
***********************************************************************************/
#ifndef ASK_USER_TEXT_WINDOW_HPP
#define ASK_USER_TEXT_WINDOW_HPP

#include <ros/ros.h>
#include <QtGui>
#include <QtGui/QWidget>
#include <QHBoxLayout>
#include <stdio.h>
#include <stdlib.h>

#include "std_msgs/String.h"
#include "std_msgs/UInt32.h"

class AskUserTextWindow : public QWidget
{
	Q_OBJECT

public:
	AskUserTextWindow( QWidget* parent );
	~AskUserTextWindow();

private:  
	void CB_requestReceived( const std_msgs::String request );
	void CB_requestCancelledReceived( const std_msgs::String request );

	QWidget*	parent_;
    QLineEdit* 	text_field_;

    ros::NodeHandle         n_;
    ros::Publisher          text_entered_pub_;

    ros::Subscriber         request_sub_;
    ros::Subscriber			request_cancelled_sub_;

private Q_SLOTS:
	void CB_textEntered();
   
Q_SIGNALS:
   
};

#endif // ASK_USER_TEXT_WINDOW_HPP
