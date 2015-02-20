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
#include "rose_main/interactive_image_display/overlay/zoombar.hpp"

ZoomBar::ZoomBar()
	:OverlayItem()
    ,m_SliderValue( 10 )
	,m_HoverPlus( false )
	,m_HoverMin( false )
	,m_DraggingSlider( false )
	,m_HoverSlider( false )
{
	setAcceptHoverEvents(true);
}
//---------------------------------------------------------------------------------
//
//
QRectF ZoomBar::boundingRect() const
{
        return QRectF(0, 0, ZOOMBAR_WIDTH, ZOOMBAR_HEIGHT );
}
//---------------------------------------------------------------------------------
//
//
void ZoomBar::resetZoom()
{
    m_SliderValue = 10;
    Q_EMIT Zoom( m_SliderValue );
    update();
}
//---------------------------------------------------------------------------------
//
//
void ZoomBar::hoverMoveEvent ( QGraphicsSceneHoverEvent * event )
{
	m_HoverPlus = false;
	m_HoverMin = false;
	m_HoverSlider = false;
	if( m_PlusButtonRect.contains( event->pos().toPoint() ) )
	{
	   setCursor( QCursor( Qt::PointingHandCursor ) );
	   m_HoverPlus = true;
	}
	else if( m_MinButtonRect.contains( event->pos().toPoint() ) )
	{
	   setCursor( QCursor( Qt::PointingHandCursor ) );
	   m_HoverMin = true;
	}
	else if( m_SliderRect.contains( event->pos().toPoint() ) )
	{
	   setCursor( QCursor( Qt::OpenHandCursor ) );
	   m_HoverSlider = true;
	}
	else
	{
	   setCursor( QCursor( Qt::ArrowCursor ) );
	}
	update();
}
//---------------------------------------------------------------------------------
//
//
void ZoomBar::hoverLeaveEvent( QGraphicsSceneHoverEvent * event )
{
	m_HoverPlus = false;
	m_HoverMin = false;
	update();
}
//---------------------------------------------------------------------------------
//
//
void ZoomBar::mousePressEvent ( QGraphicsSceneMouseEvent * event )
{
	if( m_PlusButtonRect.contains( event->pos().toPoint() ) )
	{
		m_SliderValue += 2;
		TruncSlider();
		Q_EMIT Zoom( m_SliderValue );
		update();
	}
	else if( m_MinButtonRect.contains( event->pos().toPoint() ) )
	{
		m_SliderValue -= 2;
		TruncSlider();
		Q_EMIT Zoom( m_SliderValue );
	}
	else if( m_SliderRect.contains( event->pos().toPoint() ) )
	{
	   m_DraggingSlider = true;
	   setCursor( QCursor( Qt::ClosedHandCursor ) );
	}
}
//---------------------------------------------------------------------------------
//
//
void ZoomBar::TruncSlider()
{
	if( m_SliderValue < 10 )
		m_SliderValue = 10;
	if( m_SliderValue > 90 )
		m_SliderValue = 90;
}
//---------------------------------------------------------------------------------
//
//
void ZoomBar::mouseMoveEvent ( QGraphicsSceneMouseEvent * event )
{
	if( m_DraggingSlider )
	{
		int ypos = event->pos().toPoint().y();
		int p = ypos - ZOOMBAR_RECTWIDTH;
		m_SliderValue = 100-(p*100/ZOOMBAR_LINEHEIGHT);
		//
		TruncSlider();
		Q_EMIT Zoom( m_SliderValue );
		//
		update();
	}
}
//---------------------------------------------------------------------------------
//
//
void ZoomBar::mouseReleaseEvent ( QGraphicsSceneMouseEvent * event )
{
	m_DraggingSlider = false;
}
//---------------------------------------------------------------------------------
//
//
void ZoomBar::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget )
{
	// painter->setRenderHint(QPainter::Antialiasing);
	//
	QPen pen = painter->pen();
	pen.setColor( m_LineColor );
	pen.setWidth( OVERLAY_PEN_WIDTH );
	painter->setPen( pen );

	PaintPlusButton( painter );
	PaintVerticalLine( painter );
	PaintMinusButton( painter );
	PaintTicks( painter );
	PaintSlider( painter );
}
//---------------------------------------------------------------------------------
//
//
void ZoomBar::PaintPlusButton( QPainter *painter  )
{
	//
	int xpos = ZOOMBAR_WIDTH/2;
	//
	int ypos = 0;
	//
	m_PlusButtonRect.setX( xpos - ZOOMBAR_RECTWIDTH/2 );
	m_PlusButtonRect.setY( ypos );
	m_PlusButtonRect.setWidth(ZOOMBAR_RECTWIDTH);
	m_PlusButtonRect.setHeight(ZOOMBAR_RECTWIDTH);

	if( m_HoverPlus )
	{
		QBrush brush;
		brush.setStyle( Qt::SolidPattern );
		brush.setColor( QColor(0, 142, 211) );
		painter->fillRect( m_PlusButtonRect, brush );
	}

	// Draw the TOP square with the + in it.
	painter->drawRect( m_PlusButtonRect);
	// Add the +
	painter->drawLine( xpos - ZOOMBAR_SIGNWIDTH/2, ZOOMBAR_RECTWIDTH/2, xpos + ZOOMBAR_SIGNWIDTH/2, ZOOMBAR_RECTWIDTH/2 );
	painter->drawLine( xpos, ZOOMBAR_RECTWIDTH/2-ZOOMBAR_SIGNWIDTH/2, xpos, ZOOMBAR_RECTWIDTH/2+ZOOMBAR_SIGNWIDTH/2 );
	//
}
//---------------------------------------------------------------------------------
//
//
void ZoomBar::PaintVerticalLine( QPainter *painter  )
{
	//
	int xpos = ZOOMBAR_WIDTH/2;
	//
	int ypos = 0;
	//
	// Draw the vertical line on which the ticks will reside.
	painter->drawLine( xpos , ypos + ZOOMBAR_RECTWIDTH, xpos, ypos + ZOOMBAR_RECTWIDTH + ZOOMBAR_LINEHEIGHT );
}
//---------------------------------------------------------------------------------
//
//
void ZoomBar::PaintMinusButton( QPainter *painter  )
{
	//
	int xpos = ZOOMBAR_WIDTH/2;
	//
	int ypos = 0;
	//
	m_MinButtonRect.setX( xpos - ZOOMBAR_RECTWIDTH/2 );
	m_MinButtonRect.setY( ypos + ZOOMBAR_RECTWIDTH + ZOOMBAR_LINEHEIGHT );
	m_MinButtonRect.setWidth(ZOOMBAR_RECTWIDTH);
	m_MinButtonRect.setHeight(ZOOMBAR_RECTWIDTH);

	if( m_HoverMin )
	{
		QBrush brush;
		brush.setStyle( Qt::SolidPattern );
		brush.setColor( QColor(0, 142, 211) );
		painter->fillRect( m_MinButtonRect, brush );
	}

	// Draw the BTM square with the - in it.
	painter->drawRect( m_MinButtonRect );
	// Add the -
	painter->drawLine( xpos - ZOOMBAR_SIGNWIDTH/2, ypos + ZOOMBAR_RECTWIDTH + ZOOMBAR_LINEHEIGHT + ZOOMBAR_RECTWIDTH/2, xpos + ZOOMBAR_SIGNWIDTH/2, ypos + ZOOMBAR_RECTWIDTH + ZOOMBAR_LINEHEIGHT + ZOOMBAR_RECTWIDTH/2 );
}
//---------------------------------------------------------------------------------
//
//
void ZoomBar::PaintTicks( QPainter *painter )
{
	//
	int xpos = ZOOMBAR_WIDTH/2;
	//
	int ypos = 0;
	// Draw the ticks
	int tickstep = ZOOMBAR_LINEHEIGHT/10;
	int tickoffset = ypos + ZOOMBAR_RECTWIDTH;
	int ticklength = 5;
	// draw 9 ticks.......
	for( unsigned int i = 1; i < 10; i++ )
	{
		painter->drawLine( xpos - ticklength, tickoffset + (i*tickstep), xpos + ticklength,  tickoffset + (i*tickstep) );
	}
}
//---------------------------------------------------------------------------------
//
//
void ZoomBar::PaintSlider( QPainter *painter )
{
	int sliderwidth = 20;
	int xpos = ZOOMBAR_WIDTH/2;
	int ypos = ZOOMBAR_RECTWIDTH + ( (100-m_SliderValue) * ZOOMBAR_LINEHEIGHT / 100 );
	//
	m_SliderRect.setX( xpos - sliderwidth/2 );
	m_SliderRect.setY( ypos );
	m_SliderRect.setWidth(sliderwidth);
	m_SliderRect.setHeight(8);
	//
	QBrush brush = painter->brush();
	brush.setStyle( Qt::SolidPattern );
	//
	if( m_HoverSlider || m_DraggingSlider )
		brush.setColor( QColor(0, 142, 211) );
	else
		brush.setColor( Qt::darkGray );
	//
	painter->fillRect( m_SliderRect, brush );
	painter->drawRect( m_SliderRect );
}
//---------------------------------------------------------------------------------
//
//
