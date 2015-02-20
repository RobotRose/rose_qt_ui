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
#include "rose_main/interactive_image_display/overlay/default_pos_control.hpp"

DefaultPosControl::DefaultPosControl()
	:OverlayItem()
	,m_Width( 20 )
	,m_HoveredOver( false )
{
	setAcceptHoverEvents(true);
};
//---------------------------------------------------------------------------------
//
//
QRectF DefaultPosControl::boundingRect() const
{
	return QRectF(0, 0, m_Width, m_Width );
}
//---------------------------------------------------------------------------------
//
//
void DefaultPosControl::paint( QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget )
{
	//painter->setRenderHint(QPainter::Antialiasing);
	//
	QPen pen = painter->pen();
	pen.setColor( m_LineColor );
	pen.setWidth( OVERLAY_PEN_WIDTH );
	painter->setPen( pen );
	//

	int roof_w = 18;
	int chimney_w = 4;
	int home_w = 14;
	int door_w = 5;
	int h = 18;
	int door_h = 8;

	QBrush brush = painter->brush();
	brush.setColor( m_LineColor );
	brush.setStyle( Qt::NoBrush );
	painter->setBrush( brush );

	// roof
	//                 p1
	//                / \
	//              /     \
	//            /         \
	//          /             \
	//        /                 \
	//      /                     \
	//   p7 --- p6           p3 --- p2
    //        |                 |
    //        |                 |
    //        |                 |
    //        |                 |
    //        |                 |
    //     p5 ------------------- p4
	QPainterPath path_house;
	QPoint p1, p2, p3, p4, p5, p6, p7;

	p1.setX(m_Width/2);
	p1.setY(m_Width/2-h/2);

	p2.setX(m_Width/2+roof_w/2);
	p2.setY(m_Width/2);

	p3.setX(m_Width/2+home_w/2);
	p3.setY(m_Width/2);

	p4.setX(m_Width/2+home_w/2);
	p4.setY(m_Width/2+h/2);

	p5.setX(m_Width/2-home_w/2);
	p5.setY(m_Width/2+h/2);

	p6.setX(m_Width/2-home_w/2);
	p6.setY(m_Width/2);

	p7.setX(m_Width/2-roof_w/2);
	p7.setY(m_Width/2);

	path_house.moveTo( p1 );
    path_house.lineTo( p2 );
    path_house.lineTo( p3 );
    path_house.lineTo( p4 );
    path_house.lineTo( p5 );
    path_house.lineTo( p6 );
    path_house.lineTo( p7 );
    path_house.lineTo( p1 );

	if( m_HoveredOver )
	{
		QBrush brush;
		brush.setStyle( Qt::SolidPattern );
		brush.setColor( QColor(0, 142, 211) );
		painter->fillPath( path_house, brush );
	}
	painter->drawPath( path_house );

	// door
	painter->drawRect( m_Width/2-door_w+1, m_Width/2+1, door_w, door_h );

	// chimney
	painter->drawLine( m_Width/4*3-chimney_w/2, m_Width/2-h/2, m_Width/4*3-chimney_w/2, m_Width/2-h/2+1 );
	painter->drawLine( m_Width/4*3+chimney_w/2, m_Width/2-h/2, m_Width/4*3+chimney_w/2, m_Width/2-h/2+4 );
	painter->drawLine( m_Width/4*3-chimney_w/2, m_Width/2-h/2, m_Width/4*3+chimney_w/2, m_Width/2-h/2 );

}
//---------------------------------------------------------------------------------
//
//
void DefaultPosControl::hoverEnterEvent ( QGraphicsSceneHoverEvent * event )
{
   setCursor( QCursor( Qt::PointingHandCursor ) );
   m_HoveredOver = true;
   update();
}
//---------------------------------------------------------------------------------
//
//
void DefaultPosControl::hoverLeaveEvent( QGraphicsSceneHoverEvent * event )
{
   setCursor( QCursor( Qt::ArrowCursor ) );
   m_HoveredOver = false;
   update();
}
//---------------------------------------------------------------------------------
//
//
void DefaultPosControl::mousePressEvent ( QGraphicsSceneMouseEvent * event )
{
	if( m_HoveredOver )
	{
		Q_EMIT Clicked();
	}
}
//---------------------------------------------------------------------------------
//
//
