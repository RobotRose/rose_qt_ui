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
#ifndef CENTEROFDISPLAYDRAW_HPP
#define CENTEROFDISPLAYDRAW_HPP

#include <QtGui/QGraphicsObject>
#include <QtGui/QGraphicsRectItem>
#include <QtGui/QGraphicsScene>
#include <QtGui/QGraphicsView>
#include <QtGui/QPainter>
#include <QtGui/QStackedLayout>

#include "rose_main/interactive_image_display/overlay/overlay_settings.hpp"

class CenterOfDisplayDraw : public QGraphicsObject
{
public:
	CenterOfDisplayDraw();
	QRectF boundingRect() const;
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget );
	
private:
	void DrawArcs( QColor color, int width, QPainter* painter );

	static const int MID_CIRCLE_WIDTH = 8;
	static const int MID_SQUARE_WIDTH = 64;
	static const int BOUNDING_ARC_RECT_WIDTH = 40;
};


#endif /* CENTEROFDISPLAYDRAW_HPP */
