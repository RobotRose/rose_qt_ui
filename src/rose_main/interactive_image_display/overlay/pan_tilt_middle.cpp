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
#include "rose_main/interactive_image_display/overlay/pan_tilt_middle.hpp"

PanTiltMiddle::PanTiltMiddle()
	:OverlayItem()
	,m_HoveredOver( false )
	,m_Panning( false )

{
	// We need this to have interaction at all...
	setAcceptHoverEvents(true);
	// Remember the center position for easy use later.....
	m_CenterPosition.setX( WIDTH/2 );
	m_CenterPosition.setY( WIDTH/2 );
};
//---------------------------------------------------------------------------------
//
//
QRectF PanTiltMiddle::boundingRect() const
{
	return QRectF(0, 0, WIDTH, WIDTH );
}
//---------------------------------------------------------------------------------
//
//
void PanTiltMiddle::paint( QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget )
{
	//painter->setRenderHint(QPainter::Antialiasing);
	ConfigurePen( painter );
	DrawInnerCircle( painter );
	DrawOutherCircle( painter );
	DrawVector( painter );
}
//---------------------------------------------------------------------------------
//
//
void PanTiltMiddle::ConfigurePen( QPainter* painter )
{
	QPen pen = painter->pen();
	pen.setColor( m_LineColor );
	pen.setWidth( OVERLAY_PEN_WIDTH );
	painter->setPen( pen );
}
//---------------------------------------------------------------------------------
//
//
void PanTiltMiddle::DrawInnerCircle( QPainter* painter )
{
	QBrush brush = painter->brush();
	QColor fillcolor = m_LineColor;
	if( m_HoveredOver )
		fillcolor = QColor(0, 142, 211 );
	brush.setColor( fillcolor );
	brush.setStyle( Qt::SolidPattern );
	painter->setBrush( brush );
	painter->drawEllipse( WIDTH/2-MID_CIRCLE_WIDTH/2, WIDTH/2-MID_CIRCLE_WIDTH/2, MID_CIRCLE_WIDTH, MID_CIRCLE_WIDTH );
}
//---------------------------------------------------------------------------------
//
//
void PanTiltMiddle::DrawOutherCircle( QPainter* painter )
{
	QBrush brush = painter->brush();
	brush.setColor( m_LineColor );
	brush.setStyle( Qt::NoBrush );
	painter->setBrush( brush );
	painter->drawEllipse( WIDTH/2-MAIN_CIRCLE_WIDTH/2, WIDTH/2-MAIN_CIRCLE_WIDTH/2, MAIN_CIRCLE_WIDTH, MAIN_CIRCLE_WIDTH );
}
//---------------------------------------------------------------------------------
//
//
void PanTiltMiddle::DrawVector( QPainter* painter )
{
	if( m_Panning )
	{
		painter->drawLine( m_CenterPosition, m_CurrentPos );
	}
}
//---------------------------------------------------------------------------------
//
//
void PanTiltMiddle::hoverEnterEvent ( QGraphicsSceneHoverEvent * event )
{
//	// Fired only one time.... on enter..
//	setCursor( QCursor( Qt::OpenHandCursor ) );
//	m_HoveredOver = true;
//	update();
}
//---------------------------------------------------------------------------------
//
//
void PanTiltMiddle::hoverLeaveEvent( QGraphicsSceneHoverEvent * event )
{
//	// Fired only one time, when leaving....
//	setCursor( QCursor( Qt::ArrowCursor ) );
//	m_HoveredOver = false;
//	update();
}
//---------------------------------------------------------------------------------
//
//
void PanTiltMiddle::mousePressEvent ( QGraphicsSceneMouseEvent * event )
{
//	// Mouse press events are needed because we need to detect when the user starts
//	// panning..... We're only starting to pan when we're already hoovering over
//	// the circle...
//	if( m_HoveredOver )
//	{
//		// If we're already hoovering over... we change the cursor to the one
//		// we would like to see when panning...
//		setCursor( QCursor( Qt::ClosedHandCursor ) );
//		// Update the current position for the vector, if we dont, the first update will have a
//		// invalid currentpos and the vector will be invalid.
//		m_CurrentPos = event->pos().toPoint();
//		// Set the panning boolean, so the redraw alg. will know we're panning...
//		m_Panning = true;
//		update();
//	}
//	else
//	{
//		// if we're not hoovering, we clear the panning....
//		m_Panning = false;
//		update();
//	}
}
//---------------------------------------------------------------------------------
//
//
void PanTiltMiddle::mouseMoveEvent ( QGraphicsSceneMouseEvent * event )
{
//	// This event grabs the mousemovents...
//	// We need this event, because we need to vector to be redrawn when
//	// we're panning.....
//	if( m_Panning )
//	{
//		// So we're panning....
//		// Get the position on where we are....
//		m_CurrentPos = event->pos().toPoint();
//		// We calculated a new position of the vector..
//		// If we call update, a new Qt gui update is forced in the gui thread.
//		// This will then call the paint event, that updates the screen accordingly.
//		update();
//	}
}
//---------------------------------------------------------------------------------
//
//
void PanTiltMiddle::mouseReleaseEvent ( QGraphicsSceneMouseEvent * event )
{
//	m_Panning = false;
//	//
//	if( m_HoveredOver )
//	{
//	  // If we are still above the circle... then the hand must become open again...
//	  setCursor( QCursor( Qt::OpenHandCursor ) );
//	}
//	else
//	{
//	  // If we've left the circle then the arrow should become the pointing arrow again.......
//	  setCursor( QCursor( Qt::ArrowCursor ) );
//	}
//	//
//	update();
}
//---------------------------------------------------------------------------------
//
//
