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
#ifndef CURRENTROBOTPOSITION_HPP
#define CURRENTROBOTPOSITION_HPP

#include <QtCore/QTimer>
#include <QtGui/QGraphicsRectItem>
#include <QtGui/QGraphicsScene>
#include <QtGui/QGraphicsView>
#include <QtGui/QPainter>
#include <QtGui/QStackedLayout>

#include "rose_main/interactive_image_display/overlay/overlay_item.hpp"

const int CURRENT_ROBOT_POSITION_WIDTH = 40;
const int CURRENT_ROBOT_POSITION_DOT_RADIUS = 5;
const int CIRCLE_COUNT = 3;
const int CURRENT_ROBOT_POSITION_CIRCLE_RADIUS[CIRCLE_COUNT] = { 8, 12, 16 };

class CurrentRobotPosition : public OverlayItem
{
	Q_OBJECT
public:
	//
	CurrentRobotPosition();
	QRectF boundingRect() const;
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget );
	//
	virtual void SetEnabled( bool enable );
	//
protected:
	//
private:
	//
	QTimer m_UpdateTimer;
	int m_RadiusIndex;
	//
private Q_SLOTS:
	//
	void TimerTimeout();
};


#endif /* CURRENTROBOTPOSITION_HPP */
