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
#include "rose_main/interaction_window/interaction_window.hpp"

InteractionWindow::InteractionWindow( QWidget* parent )
    : parent_ ( parent )
{
    ROS_INFO("InteractionWindow::InteractionWindow");
    //setParent(parent_);

    QGridLayout* main_layout = new QGridLayout;

    // current_action_label_ = new QLabel( "Current action");
    // current_action_label_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);

    // main_layout->addWidget(current_action_label_, 0, 0);

    choices_layout_ = new QHBoxLayout;
    choices_layout_->setSizeConstraint(QLayout::SetMaximumSize);

    main_layout->addLayout(choices_layout_, 1, 0);
    
    QVBoxLayout* button_layout = new QVBoxLayout;
    button_layout->setSizeConstraint(QLayout::SetMaximumSize);

    QPushButton *done_button = new QPushButton("Done");
    button_layout->addWidget(done_button);

    QPushButton *cancel_button = new QPushButton("Cancel");
    button_layout->addWidget(cancel_button);

    QPushButton *delete_button = new QPushButton("Delete selection");
    button_layout->addWidget(delete_button);

    main_layout->addLayout(button_layout, 1, 1);

    setLayout(main_layout);

    layout()->setSizeConstraint(QLayout::SetMaximumSize);
    layout()->setContentsMargins(0, 0, 0, 0);

    connect(this, SIGNAL(scriptsTableChanged()), this, SLOT(CB_scriptsTableChanged()));
    connect(this, SIGNAL(itemTableChanged()), this, SLOT(CB_itemTableChanged()));
    connect(this, SIGNAL(actionChanged()), this, SLOT(CB_actionChanged()));
    connect(cancel_button, SIGNAL(clicked()), this, SLOT(onCancelButtonClicked()));
    connect(done_button, SIGNAL(clicked()), this, SLOT(onDoneButtonClicked()));
    connect(delete_button, SIGNAL(clicked()), this, SLOT(onDeleteButtonClicked()));

    // Publishers
    item_selected_remove_pub_    = n_.advertise<rose_ui_item_selector::item_selected>("/item_selector/remove_selected", 1, false);
    item_selected_pub_           = n_.advertise<rose_ui_item_selector::item_selected>("/item_selector/item_selected", 1, false);
    delete_selection_pub_        = n_.advertise<rose_ui_item_selector::item_selection>("/item_selector/remove_selection", 1, false);
    item_selection_pub_          = n_.advertise<rose_ui_item_selector::item_selection>("/item_selector/item_selection", 1, false);
    item_selection_finished_pub_ = n_.advertise<rose_ui_item_selector::item_selection>("/item_selector/item_selection_finished", 1, false);

    script_cancelled_pub_        = n_.advertise<std_msgs::Bool>("/script_selector/script_cancelled", 1, false);
    script_selected_pub_         = n_.advertise<rose_ui_script_selector::script_selected>("/script_selector/script_selected", 1, false);

    // Subscribers
    scripts_sub_                = n_.subscribe("/script_selector/scripts", 1, &InteractionWindow::CB_scriptsReceived, this);;
    tables_sub_                 = n_.subscribe("/item_selector/items", 1, &InteractionWindow::CB_itemsReceived, this);

    script_manager_action_sub_  = n_.subscribe("/script_manager/current_action", 1, &InteractionWindow::CB_currentActionChanged, this); 

    ROS_INFO("Has mouse tracking set to %s ", hasMouseTracking() ? "ON" : "OFF");

    initializeTables();
}

InteractionWindow::~InteractionWindow()
{
}

void InteractionWindow::removeFirstWidget()
{
    ROS_INFO("InteractionWindow::removeFirstWidget()"); 
    QLayoutItem* item = choices_layout_->takeAt( 0 );
    delete item->widget();
    delete item;
}

void InteractionWindow::removeAllWidgets()
{
    ROS_INFO("InteractionWindow::removeAllWidgets()");
    QLayoutItem* item;
    while ( ( item = choices_layout_->takeAt( 0 ) ) != NULL )
        removeFirstWidget();
}

void InteractionWindow::addTable( InteractionTableList* list )
{
    ROS_INFO("InteractionWindow::addTable");
    choices_layout_->addWidget( list );
}

void InteractionWindow::initializeTables()
{
    ROS_INFO("InteractionWindow::initializeTables::begin");
    scriptlist_ = new InteractionTableList( this, InteractionTableList::eScriptList );
    scriptlist_->setTableSelectionMode( eSingleSelection );
    addTable( scriptlist_ );

    InteractionTableList* params;
    for ( int i = 0 ; i < INTERACTIONWINDOW_NPARAMTABLES ; i++)
    {
        params = new InteractionTableList( this, InteractionTableList::eParamList, i );
        paramlist_.push_back( params );
        addTable( params );
    }    

    ROS_INFO("InteractionWindow::initializeTables::end");
}

void InteractionWindow::itemActivated( QTableWidgetItem* item )
{
    ROS_INFO("InteractionWindow::itemActivated");
    
    InteractionTableList* list = (InteractionTableList*) QObject::sender();

    if ( list->get_listtype() == InteractionTableList::eScriptList )
    {
        rose_ui_script_selector::script_selected selection;
        selection.row_nr = item->row();
        script_selected_pub_.publish(selection);
    }

    if ( list->get_listtype() == InteractionTableList::eParamList )
    {
        rose_ui_item_selector::item_selected selection;
        selection.column = list->get_position();
        selection.row    = item->row();
        item_selected_pub_.publish(selection);
    }
}

void InteractionWindow::itemEntered( QTableWidgetItem* item )
{
    ROS_INFO("InteractionWindow::itemEntered");
    InteractionTableList* list = (InteractionTableList*) QObject::sender();
    rose_ui_item_selector::item_selected selection;

    if ( list->get_listtype() == InteractionTableList::eScriptList )
    {
        rose_ui_script_selector::script_selected selection;
        selection.row_nr = item->row();
    }

    if ( list->get_listtype() == InteractionTableList::eParamList )
    {
        selection.column = list->get_position();
        selection.row    = item->row();

        ROS_INFO("Entered item(%d, %d)", selection.column, selection.row);
        // Hovering above an item
        if ( selection.column == 1 )
        {
            //QTableWidgetItem* selected_item = paramlist_[1]->item(selection.row, 0);

            // show deletion cross
            
            // remove previous cross (if present)

        }        
    }
}

void InteractionWindow::onCancelButtonClicked()
{
    std_msgs::Bool cancelled;
    script_cancelled_pub_.publish(cancelled);
}

void InteractionWindow::onDoneButtonClicked()
{
    //! @todo MdL: fill selection
    rose_ui_item_selector::item_selection selection;
    item_selection_finished_pub_.publish( selection );
}

void InteractionWindow::onDeleteButtonClicked()
{
    rose_ui_item_selector::item_selection selection;
    delete_selection_pub_.publish( selection );
}

void InteractionWindow::CB_currentActionChanged( const std_msgs::StringConstPtr& action_message )
{
    sync_action_.lock();
    ROS_INFO("InteractionWindow::CB_currentActionChanged");
    
    // current_action_ = action_message->data;
    // current_action_label_->setText( QString::fromStdString(current_action_) );

    Q_EMIT actionChanged();

    sync_action_.unlock();
}

void InteractionWindow::CB_actionChanged()
{
    if ( sync_action_.timed_lock(boost::posix_time::milliseconds(250)) )
    {

        sync_action_.unlock();
    }
    else
    {
        ROS_DEBUG("InteractionWindow::CB_actionChanged: timed_lock timed out");
    }
}

void InteractionWindow::CB_scriptsReceived( const rose_ui_script_selector::scripts::ConstPtr& received_scripts )
{
    sync_table_.lock();

    received_scripts_ = received_scripts;

    Q_EMIT scriptsTableChanged();

    sync_table_.unlock();
}

void InteractionWindow::CB_itemsReceived( const rose_ui_item_selector::items::ConstPtr& received_items )
{
    sync_table_.lock();
    
    received_items_ = received_items;

    Q_EMIT itemTableChanged();

    sync_table_.unlock();
}

void InteractionWindow::CB_scriptsTableChanged()
{
    ROS_INFO("InteractionWindow::CB_scriptstableChanged()::begin");
    if ( sync_table_.timed_lock(boost::posix_time::milliseconds(250)) )
    {
        std::vector<std::string> received_scripts = received_scripts_->scripts;

        scriptlist_->clearTable();
        // ROS_INFO("InteractionWindow::CB_scriptstableChanged()::cleared scriptlist");

        for ( std::vector<std::string>::iterator script = received_scripts.begin() ; script != received_scripts.end() ; script++ )
        {
            InteractionTableItem* item = new InteractionTableItem(0, QString::fromStdString(*script));

            scriptlist_->addItem(item);
            // ROS_INFO("InteractionWindow::CB_scriptstableChanged()::added item");
        }

        giveItemsBackgroundColors(scriptlist_);

        sync_table_.unlock();
    }
    else
    {
        ROS_DEBUG("InteractionWindow::CB_scriptstableChanged(): timed_lock timed out");
    }
    ROS_INFO("InteractionWindow::CB_scriptstableChanged()::end");
}

void InteractionWindow::CB_itemTableChanged()
{
    if ( sync_table_.timed_lock(boost::posix_time::milliseconds(250)) )
    {
        ROS_INFO("InteractionWindow::CB_itemTableChanged::begin");

        // ROS_INFO("size: %i", (int)received_items_->tables.size());
        if ( received_items_->tables.size() > 2 ) 
        {
            // Only 2 tables are expected
            sync_table_.unlock();
            return;
        }

        
        for ( int i = 0 ; i < received_items_->tables.size() ; i++ )
        {
            // ROS_INFO("InteractionWindow::CB_itemTableChanged::for-loop1");
            paramlist_[i]->clearTable();

            for ( int j = 0 ; j < received_items_->tables[i].values.size() ; j++ )
            {
                // ROS_INFO("InteractionWindow::CB_itemTableChanged::for-loop2");
                InteractionTableItem* item = new InteractionTableItem(0, QString::fromStdString(received_items_->tables[i].values[j]));
                
                paramlist_[i]->addItem(item);
            }
            // ROS_INFO("inbetween loops");

            for ( int j = 0 ; j < received_items_->current_selection[i].values.size() ; j++ )
            {
                // ROS_INFO("InteractionWindow::CB_itemTableChanged::for-loop3");
                int item_nr = received_items_->current_selection[i].values[j];
                paramlist_[i]->setRowSelected(item_nr);
            }
            
            // ROS_INFO("after loops");
            paramlist_[i]->setTableSelectionMode( (eSelectionMode)received_items_->selection_mode[i]);
            paramlist_[i]->setMouseTracking(true);
            giveItemsBackgroundColors(paramlist_[i]);

            if ( i == received_items_->tables.size()-1) //Last table
                // Set focus to last selected item
                paramlist_[i]->scrollTo(received_items_->last_selected_item);
        }

        sync_table_.unlock();
    }
    else
    {
        ROS_DEBUG("InteractionWindow::CB_itemTableChanged: timed_lock timed out");
    }
    // ROS_INFO("InteractionWindow::CB_itemTableChanged::end");
}

void InteractionWindow::giveItemsBackgroundColors( InteractionTableList* list )
{
    QColor background_color = list->palette().color(QWidget::backgroundRole());

    for ( int i = 0 ; i < list->rowCount() ; i++ )
        if ( i % 2 == 0 )
            //! @todo MdL: Get current background color.
            list->item(i,0)->setBackgroundColor(background_color.lighter(110)); // 110% lighter
}