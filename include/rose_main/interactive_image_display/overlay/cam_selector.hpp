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
#ifndef CAMSELECTOR_HPP
#define CAMSELECTOR_HPP

#include <iostream>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/String.h>

#include <QtGui/QCursor>
#include <QtGui/QGraphicsObject>
#include <QtGui/QPainter>

#include "rose_main/interactive_image_display/overlay/overlay_item.hpp"
#include "rose_main/interactive_image_display/overlay/overlay_settings.hpp"

class CamSelector : public OverlayItem
{
	//
	Q_OBJECT
		//
	public:
		CamSelector(LRCameraSelector selector);
		//
		QRectF boundingRect() const;
		//
		void paint( QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget );
		LRCameraSelector CameraSelected() { return m_WhichCameraSelector; };
		void SetEnable( bool enable ) {  };
		//
	protected:
		//
		virtual void hoverEnterEvent ( QGraphicsSceneHoverEvent * event );
		virtual void hoverLeaveEvent( QGraphicsSceneHoverEvent * event );
		virtual void mousePressEvent ( QGraphicsSceneMouseEvent * event );
		//
	private:
		//
		void InitialiseTrianglePoints(QPoint &p1, QPoint &p2, QPoint &p3);
		void InitialiseText(QString &text);
		//
		QColor m_Color;
		//
		int m_Width;
		bool m_HoveredOver;

		LRCameraSelector m_WhichCameraSelector;

		static const int m_TriangleWidth = 26;
		static const int m_TriangleHeight = 30;

		ros::Publisher left_arm_select;
		ros::Publisher right_arm_select;


Q_SIGNALS:

		void Clicked( LRCameraSelector newcam );
};

#endif /* CAMSELECTOR_HPP */
