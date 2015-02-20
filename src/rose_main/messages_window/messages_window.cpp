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
#include "rose_main/messages_window/messages_window.hpp"

MessagesWindow::MessagesWindow( QWidget* parent )
    : parent_ ( parent )
{   
    ROS_INFO("MessagesWindow::MessagesWindow");

    text_list_ = new QListWidget;
    text_list_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    text_list_->setSelectionMode(QAbstractItemView::NoSelection);

    QHBoxLayout* layout = new QHBoxLayout;

    layout->addWidget(text_list_);

    setLayout(layout);

    // Subscribe
    text_sub_                = n_.subscribe("/messages_window/message", 1, &MessagesWindow::CB_newMessage, this);
    action_sub_              = n_.subscribe("/messages_window/action", 1, &MessagesWindow::CB_newAction, this);
    warning_sub_             = n_.subscribe("/messages_window/warning", 1, &MessagesWindow::CB_newWarning, this);

    ROS_INFO("MessagesWindow::MessagesWindow::end");

}

MessagesWindow::~MessagesWindow()
{
}

QString MessagesWindow::formatMessage( std::string message )
{
    QString return_message = "[" + QTime::currentTime().toString() + "] " + QString(message.c_str());
    return return_message;
}

void MessagesWindow::addToList( QListWidgetItem* item )
{
   text_list_->addItem(item);
   reformat();
}

void MessagesWindow::reformat()
{
    while (text_list_->count() > 50) // max nr of items
        text_list_->takeItem(0);

    text_list_->setCurrentRow(text_list_->count() - 1);
}

void MessagesWindow::CB_newMessage( const std_msgs::String request )
{
    ROS_INFO("MessagesWindow::CB_newMessage");
    QListWidgetItem* item = new QListWidgetItem(formatMessage(request.data));
    addToList(item);
}

void MessagesWindow::CB_newAction( const std_msgs::String request )
{
    ROS_INFO("MessagesWindow::CB_newAction");
    QListWidgetItem* item = new QListWidgetItem(formatMessage(request.data));
    item->setForeground(Qt::darkGreen);
    addToList(item);
}

void MessagesWindow::CB_newWarning( const std_msgs::String request )
{
    ROS_INFO("MessagesWindow::CB_newWarning");
    QListWidgetItem* item = new QListWidgetItem(formatMessage(request.data));
    item->setForeground(Qt::red);
    addToList(item);
}

