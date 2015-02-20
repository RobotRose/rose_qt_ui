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
#include "rose_main/ask_user_text_window/ask_user_text_window.hpp"

AskUserTextWindow::AskUserTextWindow( QWidget* parent )
    : parent_ ( parent )
{
    ROS_INFO("AskUserTextWindow::AskUserTextWindow");
    
    text_field_ = new QLineEdit;
    text_field_->setEnabled(false);

    QHBoxLayout* layout = new QHBoxLayout;

    layout->addWidget(text_field_);

    setLayout(layout);

    connect(text_field_, SIGNAL(returnPressed()), this, SLOT(CB_textEntered()));

    // Publishers
    text_entered_pub_           = n_.advertise<std_msgs::String>("/text_selector/text_input", 1, false);

    // Subscribe
    request_sub_                = n_.subscribe("/text_selector/request", 1, &AskUserTextWindow::CB_requestReceived, this);
    request_cancelled_sub_      = n_.subscribe("/text_selector/request_cancelled", 1, &AskUserTextWindow::CB_requestCancelledReceived, this);
}

AskUserTextWindow::~AskUserTextWindow()
{
}

void AskUserTextWindow::CB_textEntered()
{
	std_msgs::String text;
	text.data = text_field_->text().toLocal8Bit().constData();
	text_entered_pub_.publish( text );

    text_field_->setStyleSheet("QLineEdit{background-color: white; color: black;}");
	text_field_->setEnabled(false);
    text_field_->setText(QString(""));
}

void AskUserTextWindow::CB_requestReceived( const std_msgs::String request )
{
    text_field_->setText(QString(request.data.c_str()));
    text_field_->setStyleSheet("QLineEdit{background-color: green; color: white;}");
	text_field_->setEnabled(true);
}

void AskUserTextWindow::CB_requestCancelledReceived( const std_msgs::String cancel)
{
    text_field_->setText(QString(cancel.data.c_str()));
    text_field_->setStyleSheet("QLineEdit{background-color: white; color: black;}");
    text_field_->setEnabled(false);
}