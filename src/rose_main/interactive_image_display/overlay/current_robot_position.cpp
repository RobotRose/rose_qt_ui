/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/01/17
* 		- File created.
*
* Description:
*	description
* 
***********************************************************************************/
#include "rose_main/interactive_image_display/overlay/current_robot_position.hpp"

CurrentRobotPosition::CurrentRobotPosition(  )
:OverlayItem()
{
	m_UpdateTimer.setInterval( 250 );
	connect( &m_UpdateTimer, SIGNAL( timeout () ), this, SLOT(TimerTimeout() ) );
}
//---------------------------------------------------------------------------------
//
//
QRectF CurrentRobotPosition::boundingRect() const
{
  return QRectF(0, 0, CURRENT_ROBOT_POSITION_WIDTH, CURRENT_ROBOT_POSITION_WIDTH);
}
//---------------------------------------------------------------------------------
//
//
void CurrentRobotPosition::SetEnabled( bool enable )
{
	OverlayItem::SetEnabled( enable );
	//
	if( isEnabled() )
		m_UpdateTimer.start();
	else
		m_UpdateTimer.stop();
}
//---------------------------------------------------------------------------------
//
//
void CurrentRobotPosition::paint( QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget )
{
	painter->setRenderHint(QPainter::Antialiasing);
	//
	QPen pen = painter->pen();
	pen.setWidth(2);
	pen.setColor( Qt::red );
	painter->setPen( pen );

	int w = CURRENT_ROBOT_POSITION_WIDTH;
	int hw = CURRENT_ROBOT_POSITION_WIDTH/2;
	QPainterPath path;
	path.addEllipse( QPoint(hw, hw), 4, 4 );
	painter->fillPath( path, QBrush( Qt::red ) );
	//
	for( unsigned int i = 0; i < m_RadiusIndex; i++ )
	{
		painter->drawEllipse( QPoint(hw, hw), CURRENT_ROBOT_POSITION_CIRCLE_RADIUS[i], CURRENT_ROBOT_POSITION_CIRCLE_RADIUS[i] );
	}
}
//---------------------------------------------------------------------------------
//
//
void CurrentRobotPosition::TimerTimeout()
{
	// increase the number of circles that will be drawn and trunc to max, if larger, start all over with no circle at all.
	if( m_RadiusIndex++ > CIRCLE_COUNT )
		m_RadiusIndex = 0;
	// Will schedule a new paint event by Qt thread.
	update();
}
