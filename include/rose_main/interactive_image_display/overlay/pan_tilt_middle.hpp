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
#ifndef PAN_TILT_MIDDLE_HPP
#define PAN_TILT_MIDDLE_HPP

#include <QtGui/QCursor>
#include <QtGui/QGraphicsObject>
#include <QtGui/QGraphicsSceneHoverEvent>
#include <QtGui/QPainter>

#include "rose_main/interactive_image_display/overlay/overlay_item.hpp"
#include "rose_main/interactive_image_display/overlay/overlay_settings.hpp"

class QRectF;
//
class PanTiltMiddle : public OverlayItem
{
	//
	Q_OBJECT
	//
public:
	//
	PanTiltMiddle();
	QRectF boundingRect() const;
	void paint( QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget );
	//
protected:
	//
	virtual void hoverEnterEvent ( QGraphicsSceneHoverEvent * event );
	virtual void hoverLeaveEvent( QGraphicsSceneHoverEvent * event );
	virtual void mousePressEvent ( QGraphicsSceneMouseEvent * event );
	virtual void mouseMoveEvent ( QGraphicsSceneMouseEvent * event );
	virtual void mouseReleaseEvent ( QGraphicsSceneMouseEvent * event );
	//
private:
	//
	void ConfigurePen( QPainter* painter );
	void DrawInnerCircle( QPainter* painter );
	void DrawOutherCircle( QPainter* painter );
	void DrawVector( QPainter* painter );
	
	static const int MID_CIRCLE_WIDTH = 8;
	static const int MAIN_CIRCLE_WIDTH = 24;
	static const int WIDTH = 40;

	bool m_HoveredOver;
	bool m_Panning;
	QPoint m_CurrentPos;
	QPoint m_CenterPosition;
	int m_DeltaX;
	int m_DeltaY;
};

#endif /* PAN_TILT_MIDDLE_HPP */
