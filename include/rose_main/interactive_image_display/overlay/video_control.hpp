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
#ifndef VIDEOCONTROL_HPP
#define VIDEOCONTROL_HPP

#include <QtGui/QCursor>
#include <QtGui/QPainter>
#include <QtGui/QPainterPath>

#include "rose_main/interactive_image_display/overlay/overlay_item.hpp"
#include "rose_main/interactive_image_display/overlay/overlay_settings.hpp"

class VideoControl : public OverlayItem
{
	//
	Q_OBJECT
	//
public:
	VideoControl();
	//
	QRectF boundingRect() const;
	//
	void paint( QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget );
	void enable();
	void disable();
	//
protected:
	//
	virtual void hoverEnterEvent ( QGraphicsSceneHoverEvent * event );
	virtual void hoverLeaveEvent( QGraphicsSceneHoverEvent * event );
	virtual void mousePressEvent ( QGraphicsSceneMouseEvent * event );
	//
private:
	//
	int m_Width;
	bool m_enabled;
	bool m_HoveredOver;
};

#endif /* VIDEOCONTROL_HPP */
