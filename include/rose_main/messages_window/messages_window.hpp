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
#ifndef MESSAGES_WINDOW_HPP
#define MESSAGES_WINDOW_HPP

#include <ros/ros.h>
#include <QtGui>
#include <QtGui/QWidget>
#include <QHBoxLayout>
#include <QTime>
#include <stdio.h>
#include <stdlib.h>

#include "std_msgs/String.h"
#include "std_msgs/UInt32.h"

class MessagesWindow : public QWidget
{
	Q_OBJECT

  public:
	MessagesWindow( QWidget* parent );
	~MessagesWindow();

  private:  
	void 	CB_newAction( const std_msgs::String message );
	void 	CB_newMessage( const std_msgs::String message );
	void 	CB_newWarning( const std_msgs::String message );
	void 	reformat();
	QString formatMessage( std::string message );
	void 	addToList( QListWidgetItem* item );

	QWidget*	parent_;
    QTextEdit* 	text_field_;
    QListWidget* text_list_;

    ros::NodeHandle         	n_;
    ros::Subscriber         	text_sub_;   
    ros::Subscriber         	test_sub_;   
    ros::Subscriber         	action_sub_;   
    ros::Subscriber         	warning_sub_;   

    std::vector<std::string> 	messages_;
};

#endif // MESSAGES_WINDOW_HPP
