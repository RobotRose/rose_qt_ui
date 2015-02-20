/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Loy van beek
*	Date  : 2014/08/28
* 		- File created.
*
* Description:
*	An overlay for a waypoint. Consists of an arrow and a text
*
***********************************************************************************/
#ifndef WAYPOINT_HPP
#define WAYPOINT_HPP

#include <QtGui/QCursor>
#include <QtGui/QPainter>

#include <ros/ros.h>
#include "rose_common/common.hpp"
#include "rose_main/interactive_image_display/overlay/overlay_item.hpp"
#include "rose_main/interactive_image_display/overlay/overlay_settings.hpp"

#define MIN_WAYPOINT_FONTSIZE 8

class WaypointsOverlay : public OverlayItem
{
    //
    Q_OBJECT
    //
public:
    WaypointsOverlay();
    //
    QRectF boundingRect() const;
    //
    void paint( QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget );
    //
    void setWaypoints ( std::vector< std::tuple<float, float, float, float, QString> > waypoints);
    //
protected:
    //
//    virtual void hoverEnterEvent ( QGraphicsSceneHoverEvent * event );
//    virtual void hoverLeaveEvent( QGraphicsSceneHoverEvent * event );
//    virtual void mousePressEvent ( QGraphicsSceneMouseEvent * event );

    virtual void drawArrow (  QPainter *painter, float x, float y, float length, float theta, QString name);
    virtual void drawArrow (  QPainter *painter, std::tuple<float, float, float, float, QString> arrow);

    //
private:
    //
    QColor m_Color;

    static const int IMAGE_WIDTH = 640;
    static const int IMAGE_HEIGHT = 480;

    int m_Width;
    int m_Height;
    std::vector< std::tuple<float, float, float, float, QString> > m_Waypoints;
};

#endif /* WAYPOINT_HPP */
