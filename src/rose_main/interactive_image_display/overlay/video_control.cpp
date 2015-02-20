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
#include "rose_main/interactive_image_display/overlay/video_control.hpp"

VideoControl::VideoControl()
	:OverlayItem()
	,m_Width( 20 )
    ,m_enabled ( true )
    ,m_HoveredOver( false )
{
	setAcceptHoverEvents(true);
};
//---------------------------------------------------------------------------------
//
//
void VideoControl::enable()
{
	m_enabled = true;
	update();
}
//---------------------------------------------------------------------------------
//
//
void VideoControl::disable()
{
	m_enabled = false;
	update();
}
//---------------------------------------------------------------------------------
//
//
QRectF VideoControl::boundingRect() const
{
	return QRectF(0, 0, m_Width, m_Width );
}
//---------------------------------------------------------------------------------
//
//
void VideoControl::paint( QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget )
{
	//painter->setRenderHint(QPainter::Antialiasing);
	//
	QPen pen = painter->pen();
	pen.setColor( m_LineColor );
	pen.setWidth( OVERLAY_PEN_WIDTH );
	painter->setPen( pen );
	//
	int large_roll = 8;
	int small_roll = 7;
	int small_roll_offset = large_roll-small_roll; // small_roll is drawn lower than large_roll, this is the size of that offset
	int body_width = 14;
	int body_height = 8;
	int lens_width = 4;
    int total_width = body_width + lens_width;
    int total_height = body_height + large_roll;

	QBrush brush = painter->brush();
	brush.setColor( m_LineColor );
	brush.setStyle( Qt::NoBrush );
	painter->setBrush( brush );

	QPainterPath path_camera;
	QPainterPath path_lens;

	// body
	path_camera.addRect( m_Width/2-total_width/2, m_Width/2+total_height/2-body_height, body_width, body_height );
	// large film roll
	path_camera.addEllipse( m_Width/2-total_width/2, m_Width/2-total_height/2, large_roll, large_roll );
	// small film roll
	path_camera.addEllipse( m_Width/2-total_width/2+large_roll-1, m_Width/2-total_height/2+small_roll_offset, small_roll, small_roll );


	//          p2
	//          /|
	//         / |
	//        /  |
	//   p1  /   |
	//      |    |
	//      |    |
	//      |    |
	//   p4  \   |
	//        \  |
	//         \ |
	//          \|
	//          p3


	QPoint p1, p2, p3, p4;

    // rightmost pos - width lens
	p1.setX(m_Width/2+total_width/2-lens_width);
	// 2 lower than top of camera body
    p1.setY(m_Width/2+total_height/2-body_height+2);

    // rightmost pos - width lens
    p4.setX(m_Width/2+total_width/2-lens_width);
    // 2 higher than bottom
    p4.setY(m_Width/2+total_height/2-2);

    // rightmost pos - width lens
    p2.setX(m_Width/2+total_width/2);
    // 2 lower than top of camera body
    p2.setY(m_Width/2+total_height/2-body_height);

    // rightmost pos - width lens
	p3.setX(m_Width/2+total_width/2);
	// bottom
    p3.setY(m_Width/2+total_height/2);

    path_lens.moveTo( p1 );
    path_lens.lineTo( p2 );
    path_lens.lineTo( p3 );
    path_lens.lineTo( p4 );
    path_lens.lineTo( p1 );

	if( m_HoveredOver )
	{
		QBrush brush;
		brush.setStyle( Qt::SolidPattern );
		brush.setColor( QColor(0, 142, 211) );
		painter->fillPath( path_camera, brush );
		painter->fillPath( path_lens, brush );
	}
	painter->drawPath( path_camera );
	painter->drawPath( path_lens );

    if (!m_enabled)
    {
    	painter->drawLine( m_Width/2-total_width/2, m_Width/2+total_height/2, m_Width/2+total_width/2, m_Width/2-total_height/2 );
    }
}
//---------------------------------------------------------------------------------
//
//
void VideoControl::hoverEnterEvent ( QGraphicsSceneHoverEvent * event )
{
   setCursor( QCursor( Qt::PointingHandCursor ) );
   m_HoveredOver = true;
   update();
}
//---------------------------------------------------------------------------------
//
//
void VideoControl::hoverLeaveEvent( QGraphicsSceneHoverEvent * event )
{
   setCursor( QCursor( Qt::ArrowCursor ) );
   m_HoveredOver = false;
   update();
}
//---------------------------------------------------------------------------------
//
//
void VideoControl::mousePressEvent ( QGraphicsSceneMouseEvent * event )
{
	m_enabled = !m_enabled;
	update();
}
//---------------------------------------------------------------------------------
//
//

