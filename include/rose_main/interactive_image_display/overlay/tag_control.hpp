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
#ifndef TAG_CONTROL_HPP
#define TAG_CONTROL_HPP

#include <QtGui/QCursor>
#include <QtGui/QPainter>

#include "rose_main/interactive_image_display/overlay/overlay_item.hpp"
#include "rose_main/interactive_image_display/overlay/overlay_settings.hpp"

class TagControl : public OverlayItem
{
	//
	Q_OBJECT
	//
public:
	TagControl();
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
private:
	//
	QColor m_Color;
	//
	int m_Width;
	//
	bool m_enabled;
};

#endif /* TAG_CONTROL_HPP */
