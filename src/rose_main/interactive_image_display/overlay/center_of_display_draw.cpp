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
#include "rose_main/interactive_image_display/overlay/center_of_display_draw.hpp"

CenterOfDisplayDraw::CenterOfDisplayDraw(  )
	:QGraphicsObject()
{

}
//---------------------------------------------------------------------------------
//
//
QRectF CenterOfDisplayDraw::boundingRect() const
{
  return QRectF(0, 0, MID_SQUARE_WIDTH, MID_SQUARE_WIDTH);
}
//---------------------------------------------------------------------------------
//
//
void CenterOfDisplayDraw::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget )
{
	painter->setRenderHint(QPainter::Antialiasing);
	//
	DrawArcs( Qt::black, OVERLAY_PEN_WIDTH + 2, painter );
	DrawArcs( Qt::white, OVERLAY_PEN_WIDTH + 0, painter );
}
//---------------------------------------------------------------------------------
//
//
void CenterOfDisplayDraw::DrawArcs( QColor color, int width, QPainter* painter )
{
	QPen pen = painter->pen();
	pen.setColor( color );
	pen.setWidth( width );
	painter->setPen( pen );
	//
	// Draw the inner circle....
	painter->drawEllipse( MID_SQUARE_WIDTH/2-MID_CIRCLE_WIDTH/2, MID_SQUARE_WIDTH/2-MID_CIRCLE_WIDTH/2, MID_CIRCLE_WIDTH, MID_CIRCLE_WIDTH );
	//
	int offset = MID_SQUARE_WIDTH/2;
	//
	int startAngle;
	int spanAngle;
	//
	QRectF rectangle1( 0, 0, BOUNDING_ARC_RECT_WIDTH, BOUNDING_ARC_RECT_WIDTH);
	startAngle = 90 * 16;
	spanAngle =  90 * 16;
	painter->drawArc(rectangle1, startAngle, spanAngle);
	//
	QRectF rectangle2( 0, MID_SQUARE_WIDTH - BOUNDING_ARC_RECT_WIDTH, BOUNDING_ARC_RECT_WIDTH, BOUNDING_ARC_RECT_WIDTH);
	startAngle = 180 * 16;
	spanAngle  =  90 * 16;
	painter->drawArc(rectangle2, startAngle, spanAngle);
	//
	QRectF rectangle3( MID_SQUARE_WIDTH - BOUNDING_ARC_RECT_WIDTH, MID_SQUARE_WIDTH - BOUNDING_ARC_RECT_WIDTH, BOUNDING_ARC_RECT_WIDTH, BOUNDING_ARC_RECT_WIDTH);
	startAngle = 270 * 16;
	spanAngle  =  90 * 16;
	painter->drawArc(rectangle3, startAngle, spanAngle);
	//
	QRectF rectangle4( MID_SQUARE_WIDTH - BOUNDING_ARC_RECT_WIDTH, 0, BOUNDING_ARC_RECT_WIDTH, BOUNDING_ARC_RECT_WIDTH);
	startAngle =   0 * 16;
	spanAngle  =  90 * 16;
	painter->drawArc(rectangle4, startAngle, spanAngle);
	//

}
//---------------------------------------------------------------------------------
//
//
