//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

/*
 * CamIndicator.h
 *
 *  Created on: Oct 25, 2010
 *      Author: freddy
 */

#ifndef SENSORLED_H_
#define SENSORLED_H_

//
#include <QtGui/QGraphicsObject>
//
class SensorLed : public QGraphicsObject
{
public:
	SensorLed( int id, int minrange, int maxrange, QColor mincolor, QColor maxcolor );
	//
	QRectF boundingRect() const;
	//
	void paint( QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget );
	//
	void SetValue( int value );
	//
protected:
	//
	virtual void hoverEnterEvent( QGraphicsSceneHoverEvent * event );
private:
	//
	int m_Id;
	//
	int m_Width;
	int m_Height;
	//
	int m_MinRange;
	int m_MaxRange;
	//
	QColor m_MinRangeColor;
	QColor m_MaxRangeColor;
	//
	QColor m_CurrentColor;
};

#endif /* SENSORLED_H_ */
