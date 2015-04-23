//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

/*
 * OverviewCameraDisplay.cpp
 *
 *  Created on: Feb 16, 2011
 *      Author: nico
 */
//---------------------------------------------------------------------------------
//
//

#include "OverviewCameraDisplay.h"
//---------------------------------------------------------------------------------
//
//
using namespace std;
#include <iostream>
//
//
OverviewCameraDisplay::OverviewCameraDisplay( unsigned int componentselection, QWidget * parent  )
	: CameraDisplay( componentselection, parent )
    , x_scale_ (1.0)
    , y_scale_ (1.0)
    , current_cursor_pos_( -1.0, -1.0)
{
    cout << "OverviewCameraDisplay instantiation\n";
    x1 = -1;                                    // "no left mouse button pressed" state 
    this->setMouseTracking(true);              // Mouse events fire even when no button is pressed
    ros::NodeHandle nh;                                                 //ToDo determine whether placing this elsewhere so nh is not instantiated so frequently is "better"
    rect_pub    = nh.advertise<rose_ui_overview_camera::selection>("/overview_camera/selection", 1, false);       //ToDo same question as for nh    
    point_pub   = nh.advertise<rose_ui_overview_camera::selection>("/overview_camera/point_selected", 1, false);       //ToDo same question as for nh    

    QGraphicsRectItem* box;
    for ( int i = 0 ; i < 20 ; i++ )
    { 
        QBrush  brush(Qt::NoBrush);
        QPen    pen(Qt::green);
            
        pen.setWidth(2);

        box = new QGraphicsRectItem();
        box = this->scene()->addRect(-5, -5, 2, 2, pen, brush);
        box->setVisible(false);
        box->setOpacity(0.45);

        bounding_boxes_.push_back(box);
    }

    bounding_boxes_sub_ = nh.subscribe("/overview_camera/bounding_boxes", 1, &OverviewCameraDisplay::CB_boundingBoxesReceived, this);
    gui_message_sub_ = nh.subscribe("/messages_window/message", 1, &OverviewCameraDisplay::CB_process_gui_message, this);
    gui_action_sub_ = nh.subscribe("/messages_window/action", 1, &OverviewCameraDisplay::CB_process_gui_action, this);
    gui_warn_sub_ = nh.subscribe("/messages_window/warning", 1, &OverviewCameraDisplay::CB_process_gui_warn, this);
}

void OverviewCameraDisplay::mouseReleaseEvent ( QMouseEvent * event )
{
    ROS_DEBUG_NAMED(ROS_NAME, "OverviewCameraDisplay::mouseReleaseEvent");
    //Propagate the event through the scene
    QGraphicsView::mouseReleaseEvent(event);

    if ( x1 != -1 ) 
    {                                   //if state is "left mouse button pressed"
        if ( ! (event->buttons() & Qt::LeftButton) ) 
        {  //.  if left mouse button is released

            //rect_pub = nh.advertise<roscomm::rect>("bounding_box", 10);       //ToDo same question as for nh
            QPoint p = event->pos();
            int x2 = p.x();
            int y2 = p.y();

            ROS_INFO("Rectangle: x1: %i; x2: %i; x3: %i; x4: %i", x1,y1,x2,y2);

            if( (x2 > x1) && (y2 > y1))    //Only send message when the rectangle is to the right and bottom of the origin.
            {
                rose_ui_overview_camera::selection rect_msg;
                rect_msg.x1 = x1 / x_scale_;
                rect_msg.y1 = y1 / y_scale_;
                rect_msg.x2 = x2 / x_scale_;
                rect_msg.y2 = y2 / y_scale_;
                rect_pub.publish(rect_msg);
                ROS_INFO("Published Rectangle(TopLeft-X;TopLeft-Y;BottomRight-X;BottomRight-Y): %i,;%i;%i;%i", x1,y1,x2,y2);

            }
            else if ( (x1 == x2) && (y1 == y2) )
            {
                rose_ui_overview_camera::selection rect_msg;
                rect_msg.x1 = x1 / x_scale_;
                rect_msg.y1 = y1 / y_scale_;
                rect_msg.x2 = x2 / x_scale_;
                rect_msg.y2 = y2 / y_scale_;
                point_pub.publish(rect_msg);
                ROS_INFO("Published point click(TopLeft-X;TopLeft-Y;BottomRight-X;BottomRight-Y): %i,;%i;%i;%i", x1,y1,x2,y2);
            }
            else
            {
                ROS_INFO("Invalid rectangle selected.");
            }
            x1 = -1;                                    // back to "no left mouse button pressed" state

            this->scene()->removeItem(selectionRectangle);
            delete selectionRectangle;
        }
    }
}

void OverviewCameraDisplay::mousePressEvent ( QMouseEvent * event )
{
    ROS_DEBUG_NAMED(ROS_NAME, "OverviewCameraDisplay::mousePressEvent");
    //Propagate the event through the scene
    QGraphicsView::mousePressEvent(event);

    //Check if the event is handled by an item in the scene
    if(!event->isAccepted())
    {
        //  if( !rcvdImg )      ToDo should we somehow veerify there is a picture to start with ;?
        //      return;

        if ( x1 == -1 ) {                                   //if old state is "no left mouse button pressed"
            if ( event->buttons() & Qt::LeftButton ) {      //.  if new state is "left mouse button pressed"
                QPoint p = event->pos();                    //.  .  remember current position (and as side effect update state)
                x1 = p.x();
                y1 = p.y();

                QBrush brush(Qt::NoBrush);
                QPen pen(Qt::green);
                pen.setWidth(2);
                selectionRectangle = this->scene()->addRect(x1, y1, 0, 0, pen, brush);
            }
        }
    }

}

void OverviewCameraDisplay::mouseMoveEvent( QMouseEvent * event)
{
    ROS_DEBUG_NAMED(ROS_NAME, "OverviewCameraDisplay::mouseMoveEvent");
    // Propagate the event through the scene
    QGraphicsView::mouseMoveEvent(event);

    if ( x1 != -1 ) {                                  // if state is "left mouse button pressed"
        if ( event->buttons() & Qt::LeftButton ) {     // .  if left mouse button is pressed
            if(event->pos().x() > x1 && event->pos().y() > y1) {
                //Redraw rectangle. This event only gets called when a mousebutton is down (because of setMouseTracking in ctor)
                selectionRectangle->setVisible(true);
                selectionRectangle->setRect(x1, y1, event->pos().x() - x1, event->pos().y() - y1); //resize rectangle
            }
            else
                selectionRectangle->setVisible(false);
        }
    }

    // for( auto& box : bounding_boxes_ )
    // {
    //     ROS_DEBUG_NAMED(ROS_NAME, "Point x: %d, y: %d", event->pos().x(), event->pos().y());
    //     QPointF point = QPointF(event->pos().x(), event->pos().y());
    //     if (box->contains(point))
    //         box->setBrush(QBrush(box->pen().color(), Qt::SolidPattern));
    //     else
    //         box->setBrush(Qt::NoBrush);
    // }

    current_cursor_pos_.setX((float)event->pos().x());
    current_cursor_pos_.setY((float)event->pos().y());
    
    this->scene()->invalidate();
}

//! @todo MdL: Make dependant on resolution
void OverviewCameraDisplay::CB_boundingBoxesReceived( const rose_ui_overview_camera::selections selections )
{
    stored_selections_ = selections;
    drawBoundingBoxes();
}

void OverviewCameraDisplay::CB_process_gui_message( const std_msgs::String str)
{
    //! @todo LvB: This seems too tighly coupled, CameraDisplay could have a method to set the subtitle txt+color
    m_OverlayItems.m_ImageDisplay->setSubtitle(str.data, Qt::white);
}

void OverviewCameraDisplay::CB_process_gui_action( const std_msgs::String str)
{
    m_OverlayItems.m_ImageDisplay->setSubtitle(str.data, Qt::green, 10, 2);
}

void OverviewCameraDisplay::CB_process_gui_warn( const std_msgs::String str)
{
    m_OverlayItems.m_ImageDisplay->setSubtitle(str.data, Qt::red, 10, 2);
}

void OverviewCameraDisplay::drawBoundingBoxes()
{   
    ROS_DEBUG_NAMED(ROS_NAME, "OverviewCameraDisplay::drawBoundingBoxes");
    int i = 0;

    x_scale_ = sceneRect().width() / stored_selections_.width;
    y_scale_ = sceneRect().height() / stored_selections_.height;

    for ( auto& selection : stored_selections_.boxes )
    {
        int min_x       = selection.x1 * x_scale_;
        int min_y       = selection.y1 * y_scale_;
        int max_x       = selection.x2 * x_scale_;
        int max_y       = selection.y2 * y_scale_;
        bool reachable  = selection.reachable;

//        ROS_DEBUG_NAMED(ROS_NAME, "Check in scene");
        if (   min_x < sceneRect().width()
            && min_y < sceneRect().height() 
            && max_x < sceneRect().width()
            && max_y < sceneRect().height() 
            && min_x > 0 
            && min_y > 0 
            && max_x > 0 
            && max_x > 0
            && i < bounding_boxes_.size()
        )
        {
//            ROS_DEBUG_NAMED(ROS_NAME, "In scene");
            bounding_boxes_.at(i)->setVisible(true);
//            ROS_DEBUG_NAMED(ROS_NAME, "Set pen");
            if ( reachable )
                bounding_boxes_.at(i)->setPen(QPen(Qt::green));
            else
                bounding_boxes_.at(i)->setPen(QPen(Qt::red));

//            ROS_DEBUG_NAMED(ROS_NAME, "Set rect");
            bounding_boxes_.at(i)->setRect( min_x, 
                                            min_y, 
                                            (max_x - min_x), 
                                            (max_y - min_y));            

            if ( bounding_boxes_.at(i)->contains(current_cursor_pos_) ) 
                bounding_boxes_.at(i)->setBrush(QBrush(bounding_boxes_.at(i)->pen().color(), Qt::SolidPattern));
            else
                bounding_boxes_.at(i)->setBrush(Qt::NoBrush); 

//            ROS_DEBUG_NAMED(ROS_NAME, "Store box");
            i++;
        }
    }

    while ( i < bounding_boxes_.size() )
    {
        bounding_boxes_.at(i)->setVisible(false);
        i++;
    }
//    ROS_DEBUG_NAMED(ROS_NAME, "Scene has %d items", (int)this->scene()->items().size());

    // this->scene()->invalidate();
}
