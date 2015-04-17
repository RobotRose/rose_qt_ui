//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

/*
 * MapDisplay.cpp
 *
 *  Created on: Aug 9, 2011
 *      Author: Jan Willem van Silfhout
 */
//---------------------------------------------------------------------------------
//
//
#include "MapDisplay.h"
//---------------------------------------------------------------------------------
//
//
using namespace std;
#include <iostream>
#include <QtGui/QMenu>
#include <QtGui/QAction>
#include <QtGui/QContextMenuEvent>

#define _USE_MATH_DEFINES
#include <math.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
//
MapDisplay::MapDisplay( unsigned int componentselection, QWidget * parent )
        : InteractiveImageDisplay(componentselection, parent)
        , m_MapDisplayNode(0, NULL)
{
    isLMousePressed = false;
    onClickPositioning2D = false;
    onClickNavigation = false;

    m_Scene.setBackgroundBrush(Qt::transparent);
    this->setMouseTracking(true);

    //Remove scrollbars
    this->setHorizontalScrollBarPolicy( Qt::ScrollBarAlwaysOff);
    this->setVerticalScrollBarPolicy( Qt::ScrollBarAlwaysOff);

    //Black lines for a primarily white background
    this->SetColorOfLineItems(Qt::black);

    //Remove context menu
    setContextMenuPolicy( Qt::NoContextMenu );

    //Initialize node
    ROS_DEBUG_NAMED(ROS_NAME, "MapDisplay::constructor calling MapDisplayNode::init");
    m_MapDisplayNode.init("http://localhost:11311/");

    geoPub = nh.advertise<rose_ui_map_display::selection>("/map_display/selection", 1);
    posPub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);
    navGoal = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);

    //Connect to the mapChanged signal of the node
    connect(&m_MapDisplayNode, SIGNAL(mapChanged()),                this, SLOT(onMapChanged()));
    connect(&m_MapDisplayNode, SIGNAL(waypointRequested()),         this, SLOT(onWaypointRequested()));
    connect(&m_MapDisplayNode, SIGNAL(waypointRequestCanceled()),   this, SLOT(onWaypointRequestCancelled()));

    Zoom(10);
    offset = QPointF(0,0);
}
//---------------------------------------------------------------------------------
//
//
void MapDisplay::mousePressEvent(QMouseEvent *event)
{
//    ROS_DEBUG_NAMED(ROS_NAME, "MapDisplay::mousePressEvent");
    //Propagate the event through the scene
    QGraphicsView::mousePressEvent(event);

    //Check if the event is handled by an item in the scene
    if(true/*!event->isAccepted()*/)
    {
        if(!isLMousePressed) //if we're in "no left mouse button pressed" state
        {
            if((event->buttons() & Qt::LeftButton) && (onClickPositioning2D || onClickNavigation)) //if left mouse button is pressed and we're in a mode to draw some arrow
            {
                isLMousePressed = true;

                QPen pen(Qt::SolidLine);
				QBrush brush(Qt::SolidLine);
                if (onClickNavigation)
                    pen.setColor(QColor(0,255,51,160));
                else
                    pen.setColor(QColor(255,51,0,127));
                pen.setWidth(3);

                dirLine = this->scene()->addLine(event->posF().x(), event->posF().y(),
                                                 event->posF().x(), event->posF().y(),
                                                 pen);
                arrowLine1 = this->scene()->addLine(event->posF().x(), event->posF().y(),
                                                    event->posF().x(), event->posF().y(),
                                                    pen);
                arrowLine2 = this->scene()->addLine(event->posF().x(), event->posF().y(),
                                                    event->posF().x(), event->posF().y(),
                                                    pen);
                // Draw ellipse where clicked
                ellipse = this->scene()->addEllipse(event->posF().x()-1, event->posF().y()-1, 2, 2, pen, brush);
            }
        }
    }
    else
    {
//        ROS_DEBUG_NAMED(ROS_NAME, "MapDisplay::mousePressEvent is handled by someone else already");
    }
}
//---------------------------------------------------------------------------------
//
//
void MapDisplay::mouseReleaseEvent(QMouseEvent *event)
{
//    ROS_DEBUG_NAMED(ROS_NAME, " MapDisplay::mouseReleaseEvent. isLMousePressed=%s, onClickNavigation=%s",
//                    isLMousePressed ? "true" : "false",
//                    onClickNavigation ? "true" : "false");

    //Propagate the event through the scene
    QGraphicsView::mouseReleaseEvent(event);

    if(isLMousePressed)
    {
        bool inArrowMode = onClickPositioning2D || onClickNavigation;
        if(!(event->buttons() & Qt::LeftButton) && inArrowMode) //if left mouse button is released and we're in a mode to draw some arrow
        {
            //Retrieve the position released.
            QPoint p = QPoint(dirLine->line().x1(), dirLine->line().y1());

            //Transform back to original map coordinates
            QTransform curInvTr = m_OverlayItems.m_ImageDisplay->transform().inverted();
            QPoint pTransformed = curInvTr.map(p);

            //Flip the point over the horizontal axis (see MapDisplayNode).
            pTransformed = QPoint(pTransformed.x(), m_CurMap.height() - pTransformed.y());

            //Verify click is inside map and on a known location
            if( pTransformed.x() >= 0 &&
                pTransformed.y() >= 0 &&
                pTransformed.x() < m_CurMap.width() &&
                pTransformed.y() < m_CurMap.height())
            {
                //Convert the scaled map pixels to full map pixels to meters
                QPoint pScaled = m_MapDisplayNode.transformScaledPixelsToFullPixels(pTransformed);         
                std::pair<float,float> meters = m_MapDisplayNode.fullPixelsToMeters(pScaled);

                //Calculate angle in radians
                float angle = atan2(dirLine->line().y2() - dirLine->line().y1(),
                                    dirLine->line().x2() - dirLine->line().x1());

                geometry_msgs::Point position;
                position.x = meters.first;
                position.y = meters.second;

                geometry_msgs::Quaternion orientation;
                orientation.x = 0;
                orientation.y = 0;
                orientation.z = sin(angle/2);
                orientation.w = -1 * cos(angle/2);

                if ( onClickNavigation )
                {
                    rose_ui_map_display::selection selection;
                    selection.x = meters.first;
                    selection.y = meters.second;
                    selection.angle = -angle;
                    //Create new navigation message
                    //! @todo: Remove
                    geometry_msgs::PoseStamped geoMsg;
                    geoMsg.header.frame_id = "/map";
                    geoMsg.pose.position = position;
                    geoMsg.pose.orientation = orientation;

                    geoPub.publish(selection);
                    ROS_INFO("Published navigational click on map (x,y): %.5f,%.5f", position.x, position.y);
                    Q_EMIT setNavGoalDone();
                }
                else
                {
                    // Create new position message
                    geometry_msgs::PoseWithCovarianceStamped posMsg;
                    posMsg.header.frame_id = "/map";
                    posMsg.pose.pose.position = position;
                    posMsg.pose.pose.orientation = orientation;

                    posPub.publish(posMsg);
                    ROS_INFO("Published 2d positioning click on map (x,y): %.5f,%.5f", position.x, position.y);                    
                    Q_EMIT setPose2dDone();
                }
            }
            else
            {
                if (m_MapDisplayNode.isKnownLocation(pTransformed.x(), pTransformed.y()) == false)
                    ROS_DEBUG("Unknown location.");
                else
                    ROS_DEBUG("Released outside map.");
            }

            isLMousePressed = false; // back to "no left mouse button pressed" state
            this->scene()->removeItem(dirLine);
            this->scene()->removeItem(arrowLine1);
            this->scene()->removeItem(arrowLine2);  
            delete dirLine;
            delete arrowLine1;
            delete arrowLine2;
            this->scene()->removeItem(ellipse);
            delete ellipse;
        }
    }
}

void MapDisplay::setNoNavigation()
{
    ROS_DEBUG_NAMED(ROS_NAME, "MapDisplay::setNoNavigation");
    onClickPositioning2D = false;
    onClickNavigation = false;

    Q_EMIT setPoseOrNavSelectionStopped();
}

void MapDisplay::set2DPositioning()
{
    ROS_DEBUG_NAMED(ROS_NAME, "MapDisplay::set2DPositioning");
    onClickPositioning2D = true;
    onClickNavigation = false;

    Q_EMIT setPose2dStarted();
}

void MapDisplay::setNavigation()
{
    ROS_DEBUG_NAMED(ROS_NAME, "MapDisplay::setNavigation");
    onClickPositioning2D = false;
    onClickNavigation = true;

    Q_EMIT setNavGoalStarted();
}

void MapDisplay::wheelEvent(QWheelEvent *event)
{
    // TODO
}

//---------------------------------------------------------------------------------
//
//
void MapDisplay::mouseMoveEvent( QMouseEvent * event)
{
    //Propagate the event through the scene
    QGraphicsView::mouseMoveEvent(event);

    //Redraw the current line
    if(isLMousePressed)
    {
        drawArrow(event->posF(), dirLine->line().p1());
    }
}
//---------------------------------------------------------------------------------
// Get the new map, transform and display it.
//
#include <boost/version.hpp>

void MapDisplay::onMapChanged()
{
    if(m_MapDisplayNode.syncroot.timed_lock(boost::posix_time::milliseconds(250)))
    {
        m_CurMap = m_MapDisplayNode.getmap(true,true);
        transformToPanAndZoom();
        m_OverlayItems.m_ImageDisplay->ShowImageFromQImage(m_CurMap);

        auto waypoints = m_MapDisplayNode.getWaypoints();
//        ROS_DEBUG_NAMED(ROS_NAME, "MapDisplay::onMapChanged: passing on %i waypoints", (int)waypoints.size());

        if(m_OverlayItems.m_WaypointsOverlay)
            m_OverlayItems.m_WaypointsOverlay->setWaypoints(waypoints);

        m_MapDisplayNode.syncroot.unlock();
    }
    else
    {
        ROS_DEBUG("MapDisplay::onMapChanged: timed_lock timed out");
    }
}

void MapDisplay::onWaypointRequested()
{
    ROS_DEBUG_NAMED(ROS_NAME, "MapDisplay::onWaypointRequested");
    setNavigation();
}

void MapDisplay::onWaypointRequestCancelled()
{
    ROS_DEBUG_NAMED(ROS_NAME, "MapDisplay::onWaypointRequestCancelled");
    setNoNavigation();
}

//---------------------------------------------------------------------------------
// Transform m_CurMap to the current pan and zoom.
//
void MapDisplay::transformToPanAndZoom()
{
    //Get center of view
    double ctX = m_OverlayItems.m_ImageDisplay->boundingRect().width() / 2;
    double ctY = m_OverlayItems.m_ImageDisplay->boundingRect().height() / 2;

    //Calculate x+y maximum scale and keep lowest to stay within bounds of view
    double xScale = this->width() / (double)m_CurMap.width();
    double yScale = this->height() / (double)m_CurMap.height();

    double newScale = xScale > yScale ? yScale : xScale;

    int xOffset = (this->width() - (m_CurMap.width() * newScale)) / 2;
    int yOffset = (this->height() - (m_CurMap.height() * newScale)) / 2;

    //Create the fitting transform with new translation and scale
    QTransform curTr = m_OverlayItems.m_ImageDisplay->transform();
    curTr.setMatrix(newScale, curTr.m12(), curTr.m13(), curTr.m21(), newScale,
                    curTr.m23(), xOffset, yOffset, curTr.m33());

    curTr.translate(ctX, ctY)               //Move scale-origin to 0,0
            .scale(zoomFactor, zoomFactor)  //Apply the zoom
            .translate(-1 * ctX, -1 * ctY); //Move back

    //Apply the offset (from the arrow buttons)
    curTr.translate(offset.x(), offset.y());
    
    //Apply the transformation
    m_OverlayItems.m_ImageDisplay->setTransform(curTr);
    if(m_OverlayItems.m_WaypointsOverlay)
    {
        m_OverlayItems.m_WaypointsOverlay->setTransform(curTr);
    }
}

//---------------------------------------------------------------------------------
// Redraw the arrow from a to b.
//
void MapDisplay::drawArrow(QPointF a, QPointF b)
{
    QPointF p1,p2,pd;
    float tangent;

    pd.setX(b.x() - a.x());
    pd.setY(b.y() - a.y());
    if(pd.x() == 0 && pd.y() == 0)
        return;

    float size = sqrt(pow(fabs(pd.x()),2) + pow(fabs(pd.y()),2))/3;

    tangent = atan2(pd.y(), pd.x());
    p1.setX(size * cos (tangent + M_PI / 7) + a.x());
    p1.setY(size * sin (tangent + M_PI / 7) + a.y());
    p2.setX(size * cos (tangent - M_PI / 7) + a.x());
    p2.setY(size * sin (tangent - M_PI / 7) + a.y());

    dirLine->setLine(b.x(), b.y(), a.x(), a.y());
    arrowLine1->setLine(a.x(), a.y(), p1.x(), p1.y());
    arrowLine2->setLine(a.x(), a.y(), p2.x(), p2.y());
}

//---------------------------------------------------------------------------------
//
//
void MapDisplay::MoveUpClicked()
{
    offset.setY(offset.y() + 80);
    transformToPanAndZoom();
}
//---------------------------------------------------------------------------------
//
//
void MapDisplay::MoveDownClicked()
{
    offset.setY(offset.y() - 80);
    transformToPanAndZoom();
}
//---------------------------------------------------------------------------------
//
//
void MapDisplay::MoveLeftClicked()
{
    offset.setX(offset.x() + 80);
    transformToPanAndZoom();
}
//---------------------------------------------------------------------------------
//
//
void MapDisplay::MoveRightClicked()
{
    offset.setX(offset.x() - 80);
    transformToPanAndZoom();
}
//---------------------------------------------------------------------------------
// Set the current zoom factor
//
void MapDisplay::Zoom( int percentage )
{
        // Calculate factor to scale the image with.
        double min = 0.0;
        double max = 10.0;
        //
        double range = max - min;

        //Change member to update on paint
        zoomFactor = min + ((double)percentage/100.0) * range;
        transformToPanAndZoom();
}
//---------------------------------------------------------------------------------
//
//
void MapDisplay::HouseClicked()
{
    //Reset the view (remove translate and scale transformations).
    offset = QPointF(0,0);
    m_OverlayItems.m_ZoomBar->resetZoom();
}
//---------------------------------------------------------------------------------
//
//
void MapDisplay::ShowMapImage( const QImage& image )
{

}
