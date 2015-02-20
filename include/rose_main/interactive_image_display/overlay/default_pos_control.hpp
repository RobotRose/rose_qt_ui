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
#ifndef DEFAULTPOSCONTROL_HPP
#define DEFAULTPOSCONTROL_HPP

#include <iostream>

#include <QtGui/QCursor>
#include <QtGui/QGraphicsObject>
#include <QtGui/QPainter>

#include "rose_main/interactive_image_display/overlay/overlay_item.hpp"
#include "rose_main/interactive_image_display/overlay/overlay_settings.hpp"

class DefaultPosControl : public OverlayItem
{
	//
	Q_OBJECT
	//
public:
	DefaultPosControl();
	//
	QRectF boundingRect() const;
	//
	void paint( QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget );
	//
protected:
	//
	virtual void hoverEnterEvent ( QGraphicsSceneHoverEvent * event );
	virtual void hoverLeaveEvent( QGraphicsSceneHoverEvent * event );
	virtual void mousePressEvent ( QGraphicsSceneMouseEvent * event );
	//
Q_SIGNALS:
	//
	void Clicked();
	//
private:
	//
	QColor m_Color;
	//
	int m_Width;
	bool m_HoveredOver;
};

#endif /* DEFAULTPOSCONTROL_HPP */
