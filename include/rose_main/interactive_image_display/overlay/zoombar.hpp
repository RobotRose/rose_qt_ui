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
#ifndef ZOOMBAR_HPP
#define ZOOMBAR_HPP

#include <iostream>

#include <QtGui/QGraphicsSceneHoverEvent>
#include <QtGui/QCursor>
#include <QtGui/QPainter>

#include "rose_main/interactive_image_display/overlay/overlay_item.hpp"
#include "rose_main/interactive_image_display/overlay/overlay_settings.hpp"

const int ZOOMBAR_WIDTH = 40;
const int ZOOMBAR_HEIGHT = 182;
const int ZOOMBAR_RECTWIDTH = 16;
const int ZOOMBAR_SIGNWIDTH = 8;
const int ZOOMBAR_LINEHEIGHT = ZOOMBAR_HEIGHT-ZOOMBAR_RECTWIDTH-ZOOMBAR_RECTWIDTH;

class ZoomBar : public OverlayItem
{
	//
	Q_OBJECT
	//
public:
	//
	ZoomBar();
	//
	QRectF boundingRect() const;
	//
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget );
	//
        void resetZoom();
protected:
	//
	virtual void hoverMoveEvent ( QGraphicsSceneHoverEvent * event );
	virtual void hoverLeaveEvent( QGraphicsSceneHoverEvent * event );
	virtual void mousePressEvent ( QGraphicsSceneMouseEvent * event );
	virtual void mouseMoveEvent ( QGraphicsSceneMouseEvent * event );
	virtual void mouseReleaseEvent ( QGraphicsSceneMouseEvent * event );
	//
private:
	//
	void PaintSlider( QPainter* painter );
	void PaintPlusButton( QPainter* painter );
	void PaintVerticalLine( QPainter* painter );
	void PaintMinusButton( QPainter* painter );
	void PaintTicks( QPainter *painter );
	//
	void TruncSlider();
	//
	int m_SliderValue;
	//
	QRect m_PlusButtonRect;
	QRect m_MinButtonRect;
	QRect m_SliderRect;

	bool m_HoverPlus;
	bool m_HoverMin;
	bool m_HoverSlider;
	bool m_DraggingSlider;

Q_SIGNALS:
	void Zoom( int percentage );

};


#endif /* ZOOMBAR_HPP */
