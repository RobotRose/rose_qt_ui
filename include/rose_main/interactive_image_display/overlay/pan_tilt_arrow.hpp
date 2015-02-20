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
#ifndef PANTILTARROW_HPP
#define PANTILTARROW_HPP

#include <iostream>
#include <QtCore/QTimer>
#include <QtGui/QCursor>
#include <QtGui/QGraphicsItem>
#include <QtGui/QPainter>
#include <QtGui/QPainterPath>
#include <QtGui/QRgb>

#include "rose_main/interactive_image_display/overlay/overlay_item.hpp"
#include "rose_main/interactive_image_display/overlay/overlay_settings.hpp"

#define PRESSDELAY_MS 250

class PanTiltArrow : public OverlayItem
{
	//
	Q_OBJECT
	//
public:
	PanTiltArrow();
	//
	QRectF boundingRect() const;
	//
	void paint( QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget );
	int height() { return m_Height; }
	//
protected:
	//
	virtual void hoverEnterEvent( QGraphicsSceneHoverEvent * event );
	virtual void hoverLeaveEvent( QGraphicsSceneHoverEvent * event );
	virtual void mousePressEvent ( QGraphicsSceneMouseEvent * event );
	virtual void mouseReleaseEvent ( QGraphicsSceneMouseEvent * event );
	//
private:
	//
	QColor m_Color;
	//
	int m_Width;
	int m_Height;
	//
	bool HoveredOn;
	//
	bool Pressed;
	bool PressedContinuous;
	QTimer PressDurationTimer;
	//
private Q_SLOTS:
	//
	void PressDurationTimerTimeOut();
	//
Q_SIGNALS:
	//
	void ButtonClicked();
	void ButtonDownLongStart();
	void ButtonDownLongStop();
	//
};

#endif /* PANTILTARROW_HPP */
