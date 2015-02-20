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
#ifndef CAMINDICATOR_HPP
#define CAMINDICATOR_HPP

#include <iostream>

#include <QtGui/QCursor>
#include <QtGui/QGraphicsObject>
#include <QtGui/QPainter>

#include "rose_main/interactive_image_display/overlay/overlay_item.hpp"
#include "rose_main/interactive_image_display/overlay/overlay_settings.hpp"

class CamIndicator : public OverlayItem
{
public:
	CamIndicator();
	//
	QRectF boundingRect() const;
	//
	void paint( QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget );
	void SetCurrentCamera( LRCameraSelector camera );
	//
protected:
	//
	//
private:
	//
	int m_Width;
	LRCameraSelector m_CameraSelected;
};

#endif /* CAMINDICATOR_HPP */
