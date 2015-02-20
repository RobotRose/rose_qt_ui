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
#include "rose_main/interactive_image_display/overlay/tag_control.hpp"

TagControl::TagControl()
	:OverlayItem()
	,m_Width( 20 )
    ,m_enabled( true )
{
	setAcceptHoverEvents(true);
};
//---------------------------------------------------------------------------------
//
//
QRectF TagControl::boundingRect() const
{
	return QRectF(0, 0, m_Width, m_Width );
}
//---------------------------------------------------------------------------------
//
//
void TagControl::paint( QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget )
{
	//painter->setRenderHint(QPainter::Antialiasing);
	//	//
	QPen pen = painter->pen();
	pen.setColor( m_LineColor );
	pen.setWidth( OVERLAY_PEN_WIDTH );
	painter->setPen( pen );
	//
	int circle_width = 20;

	QBrush brush = painter->brush();
	brush.setColor( m_LineColor );
	brush.setStyle( Qt::NoBrush );
	painter->setBrush( brush );

	painter->drawEllipse( m_Width/2-circle_width/2, m_Width/2-circle_width/2, circle_width, circle_width );

	if( !m_enabled )
	{
		painter->drawLine( m_Width/2-circle_width/2, m_Width/2-circle_width/2, m_Width/2+circle_width/2, m_Width/2+circle_width/2 );
		painter->drawLine( m_Width/2-circle_width/2, m_Width/2+circle_width/2, m_Width/2+circle_width/2, m_Width/2-circle_width/2 );
	}
}
//---------------------------------------------------------------------------------
//
//
void TagControl::hoverEnterEvent ( QGraphicsSceneHoverEvent * event )
{
   setCursor( QCursor( Qt::PointingHandCursor ) );
   update();
}
//---------------------------------------------------------------------------------
//
//
void TagControl::hoverLeaveEvent( QGraphicsSceneHoverEvent * event )
{
   setCursor( QCursor( Qt::ArrowCursor ) );
   update();
}
//---------------------------------------------------------------------------------
//
//
void TagControl::mousePressEvent ( QGraphicsSceneMouseEvent * event )
{
	m_enabled = !m_enabled;
	update();
}
//---------------------------------------------------------------------------------
//
//
