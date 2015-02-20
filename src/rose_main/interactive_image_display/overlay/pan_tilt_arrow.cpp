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
#include "rose_main/interactive_image_display/overlay/pan_tilt_arrow.hpp"

PanTiltArrow::PanTiltArrow()
	:OverlayItem()
	,m_Width( 20 )
	,m_Height( 10 )
	,HoveredOn( false )
	,Pressed( false )
	,PressedContinuous( false )
{
	setAcceptHoverEvents(true);
}
//---------------------------------------------------------------------------------
//
//
QRectF PanTiltArrow::boundingRect() const
{
	return QRectF(0, 0, m_Width, m_Height );
}
//---------------------------------------------------------------------------------
//
//
void PanTiltArrow::hoverEnterEvent( QGraphicsSceneHoverEvent * event )
{
   HoveredOn = true;
   setCursor( QCursor( Qt::PointingHandCursor ) );
   update();
}
//---------------------------------------------------------------------------------
//
//
void PanTiltArrow::hoverLeaveEvent( QGraphicsSceneHoverEvent * event )
{
   HoveredOn = false;
   setCursor( QCursor( Qt::ArrowCursor ) );
   update();
}
//---------------------------------------------------------------------------------
//
//
void PanTiltArrow::mousePressEvent ( QGraphicsSceneMouseEvent * event )
{
	if( !Pressed )
	{
		//
		Pressed = true;
		PressDurationTimer.singleShot( PRESSDELAY_MS, this, SLOT( PressDurationTimerTimeOut() ) );
	}
	else
	{
		// Stop the timer, since we dont need to time if the user still presses the button
		PressDurationTimer.stop();
		//
		Pressed = false;
		//
		PressedContinuous = false;
	}
}
//---------------------------------------------------------------------------------
//
//
void PanTiltArrow::PressDurationTimerTimeOut()
{
	if( Pressed )
	{
		PressedContinuous = true;
		Q_EMIT ButtonDownLongStart();
	}
}
//---------------------------------------------------------------------------------
//
//
void PanTiltArrow::mouseReleaseEvent ( QGraphicsSceneMouseEvent * event )
{
	if( Pressed )
	{
		if( !PressedContinuous )
		{
			// we had the button pressed, but we did not yet start the continuous pressed functionality.
			// This basically is a click then.

			Q_EMIT ButtonClicked();
		}
		else
		{
			Q_EMIT ButtonDownLongStop();
		}
	}
	// Stop the timer, since we dont need to time if the user still presses the button
	PressDurationTimer.stop();
	//
	Pressed = false;
	//
	PressedContinuous = false;
}
//---------------------------------------------------------------------------------
//
//
void PanTiltArrow::paint( QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget )
{
	//painter->setRenderHint(QPainter::Antialiasing);
	//
	QPen pen = painter->pen();

	pen.setWidth( OVERLAY_PEN_WIDTH );

	QPainterPath path;

	path.moveTo( 0, m_Height);
	path.lineTo( m_Width  , m_Height);
	path.lineTo( m_Width/2, 0);
	path.lineTo( 0, m_Height );
    pen.setColor( m_LineColor );

    painter->setPen( pen );

	//
	if( HoveredOn )
	{
		QBrush brush;
		brush.setStyle( Qt::SolidPattern );
		brush.setColor( QColor(0, 142, 211) );
		painter->fillPath( path, brush );

		painter->drawPath( path );
	}
	else
	{
		painter->drawPath( path );
	}
}
//---------------------------------------------------------------------------------
//
//
