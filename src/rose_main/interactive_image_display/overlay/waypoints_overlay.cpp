/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Loy van beek
*	Date  : 2014/08/28
* 		- File created.
*
* Description:
*	description
*
***********************************************************************************/
#include "rose_main/interactive_image_display/overlay/waypoints_overlay.hpp"

WaypointsOverlay::WaypointsOverlay()
    :OverlayItem()
    ,m_Width( IMAGE_WIDTH )
    ,m_Height( IMAGE_HEIGHT )
{
    ROS_DEBUG_NAMED(ROS_NAME, "WaypointsOverlay::WaypointsOverlay");
    setAcceptHoverEvents(true);

//    auto arrow200 = std::tuple<float, float, float, float, QString>(200.0, 200.0, 20.0, 1.57, QString("Waypoint200"));
//    auto arrow100 = std::tuple<float, float, float, float, QString>(100, 100, 20.0, 0, QString("Waypoint100"));
//    auto arrow1 = std::tuple<float, float, float, float, QString>(1, 1, 20.0, 0, QString("Waypoint1"));
//    auto arrow2 = std::tuple<float, float, float, float, QString>(2, 2, 20.0, 0, QString("Waypoint2"));

//    std::vector< std::tuple<float, float, float, float, QString> > waypoints;
//    waypoints.push_back(arrow200);
//    waypoints.push_back(arrow100);
//    waypoints.push_back(arrow1);
//    waypoints.push_back(arrow2);

//    setWaypoints(waypoints);
}
//---------------------------------------------------------------------------------
//
//
QRectF WaypointsOverlay::boundingRect() const
{
    return QRectF(0, 0, m_Width, m_Width );
}
//---------------------------------------------------------------------------------
//
//
void WaypointsOverlay::paint( QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget )
{
//    ROS_DEBUG_NAMED(ROS_NAME, "WaypointsOverlay::paint: %i waypoints", (int)m_Waypoints.size());

    for (auto &wp : m_Waypoints)
    {
        drawArrow(painter, wp);
    }
}

void WaypointsOverlay::drawArrow (  QPainter *painter, std::tuple<float, float, float, float, QString> arrow)
{
    drawArrow(painter, std::get<0>(arrow), std::get<1>(arrow), std::get<2>(arrow), std::get<3>(arrow), std::get<4>(arrow));
}

void WaypointsOverlay::drawArrow (  QPainter *painter, float x, float y, float length, float theta, QString name)
{
//    ROS_DEBUG_NAMED(ROS_NAME, "WaypointsOverlay::drawArrow: %s on (%f, %f, %f)", name.toStdString().c_str(), x, y, theta);

    int wpX = x;
    int wpY = y;

    float factor = (float)length / (float)painter->fontMetrics().width(name);
    if ((factor < 1) || (factor > 1.25))
    {
        QFont f = painter->font();
        float newPointSize = f.pointSizeF()*factor;
        if(newPointSize < MIN_WAYPOINT_FONTSIZE)
        {
            newPointSize = MIN_WAYPOINT_FONTSIZE;
        }
        f.setPointSizeF(newPointSize);
        painter->setFont(f);
    }

    painter->setRenderHint(QPainter::Antialiasing, true);

    QPen pen = painter->pen();
    pen.setColor( Qt::red );
    pen.setWidth( 1 );
    painter->setPen( pen );

    auto x_length = length * cos((double)theta);
    auto y_length = length * sin((double)theta);


    QPoint arrowOrigin      = QPoint(wpX, wpY);
    QPoint arrowHeadCenter  = QPoint(wpX+length, wpY);
    QPoint arrowHelperPt    = QPoint(wpX+0.75*length, wpY);
    QLineF arrowMain        = QLineF(arrowOrigin, arrowHeadCenter);
    QLineF helper           = QLineF(arrowOrigin, arrowHelperPt);
    arrowMain.setAngle(theta);
    helper.setAngle(theta);

    //TODO: Use Qt's transformations to do all rotations!

    QLineF arrowHeadLeft    = QLineF(arrowMain.p2(), helper.p2()); //p2 is arrowHelperPt, rotated about the arrow's origin
    arrowHeadLeft.setAngle(theta-135);
    QLineF arrowHeadRight   = QLineF(arrowMain.p2(), helper.p2()); //p2 is arrowHelperPt, rotated about the arrow's origin
    arrowHeadRight.setAngle(theta+135);

    float textYoffset = length/4;
    if(arrowMain.dy() < 0.0) //Return value is positive if y2() >= y1() (arrow points down) and negative if y2() < y1(). (arrow points up)
    {
        textYoffset *= -2.0;
    }

    QPoint textOrigin       = QPoint(wpX-length/2, wpY-textYoffset);
    painter->drawText(textOrigin, name);

    painter->drawLine(arrowMain);       //Body of the arrow
    painter->drawLine(arrowHeadLeft);   //Left side
    painter->drawLine(arrowHeadRight);  //Right side

    painter->setBrush(Qt::red);
    painter->drawEllipse(arrowOrigin, 2, 2);
}

//---------------------------------------------------------------------------------
//
//
//void WaypointsOverlay::hoverEnterEvent ( QGraphicsSceneHoverEvent * event )
//{
//   ROS_DEBUG_NAMED(ROS_NAME, "WaypointsOverlay::hoverEnterEvent");
//   setCursor( QCursor( Qt::PointingHandCursor ) );
//   update();
//}
//---------------------------------------------------------------------------------
//
//
//void WaypointsOverlay::hoverLeaveEvent( QGraphicsSceneHoverEvent * event )
//{
//    ROS_DEBUG_NAMED(ROS_NAME, "WaypointsOverlay::hoverLeaveEvent");
//   setCursor( QCursor( Qt::ArrowCursor ) );
//   update();
//}
//---------------------------------------------------------------------------------
//
//
//void WaypointsOverlay::mousePressEvent ( QGraphicsSceneMouseEvent * event )
//{
//    ROS_DEBUG_NAMED(ROS_NAME, "WaypointsOverlay::mousePressEvent");
//    update();
//}
//---------------------------------------------------------------------------------
//
//
void WaypointsOverlay::setWaypoints ( std::vector< std::tuple<float, float, float, float, QString> > waypoints)
{
    m_Waypoints = waypoints;
}
