/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2013/12/12
* 		- File created.
*
* Description:
*	Scriptmanager to manage scripts for the user interface
* 
***********************************************************************************/

#ifndef INTERACTIONWINDOW_H
#define INTERACTIONWINDOW_H

#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <QtGui>
#include <QtGui/QWidget>
#include <QHBoxLayout>
#include <QTableWidget>
#include <stdio.h>
#include <stdlib.h>

#include "boost/date_time/posix_time/posix_time.hpp"

#include "rose_ui_item_selector/item_selected.h"
#include "rose_ui_item_selector/item_selection.h"
#include "rose_ui_item_selector/items.h"

#include "rose_ui_script_selector/script_selected.h"
#include "rose_ui_script_selector/scripts.h"

#include "rose_main/interaction_window/interaction_table_list.hpp"
#include "rose_main/interaction_window/interaction_table_item.hpp"

#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#define INTERACTIONWINDOW_NPARAMTABLES 2

class InteractionWindow : public QWidget
{
	Q_OBJECT

public:
	InteractionWindow( QWidget* parent );
	~InteractionWindow();

private:
    void addTable( InteractionTableList* list );
    void removeFirstWidget();
    void removeAllWidgets();

    void initializeTables();

    void giveItemsBackgroundColors( InteractionTableList* list );

    void CB_scriptsReceived( const rose_ui_script_selector::scripts::ConstPtr& received_scripts );

    void CB_itemsReceived( const rose_ui_item_selector::items::ConstPtr& received_items );

    void CB_currentActionChanged( const std_msgs::StringConstPtr& action_message );

    QWidget*                            parent_;
    QHBoxLayout*                        choices_layout_;
    InteractionTableList*               scriptlist_;
    std::vector<InteractionTableList*>  paramlist_;
    QLabel*                             current_action_label_;
    std::vector<InteractionTableItem>   selected_items_;
    int                                 last_selection_;

    rose_ui_item_selector::items::ConstPtr      received_items_;
    rose_ui_script_selector::scripts::ConstPtr  received_scripts_;
    std::string                             current_action_;

    ros::NodeHandle         n_;
    ros::Publisher          item_selected_pub_;
    ros::Publisher          item_selected_remove_pub_;
    ros::Publisher          item_selection_pub_;
    ros::Publisher          delete_selection_pub_;
    ros::Publisher          item_selection_finished_pub_;

    ros::Publisher          script_cancelled_pub_;
    ros::Publisher          script_selected_pub_;

    ros::Subscriber         scripts_sub_;
    ros::Subscriber         tables_sub_;
    ros::Subscriber         script_manager_action_sub_;

    boost::timed_mutex      sync_table_;
    boost::timed_mutex      sync_action_;

private Q_SLOTS:
    void itemActivated( QTableWidgetItem* item );
    void itemEntered( QTableWidgetItem* item );
    void onCancelButtonClicked();
    void onDoneButtonClicked();
    void onDeleteButtonClicked();

    void CB_scriptsTableChanged();
    void CB_itemTableChanged();
    void CB_actionChanged();

Q_SIGNALS:
    void scriptsTableChanged();
    void itemTableChanged();
    void actionChanged();
    // void itemSelectionChanged();
};

#endif // INTERACTIONWINDOW_H
