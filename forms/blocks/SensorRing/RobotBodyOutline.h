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

#ifndef ROBOTBODYOUTLINE_H_
#define ROBOTBODYOUTLINE_H_

//
#include <QtGui/QGraphicsObject>
#include <QtCore/QPoint>
//
#include "SensorLocation.h"
//
class RobotBodyOutline : public QGraphicsObject
{
public:

	struct SensorConfiguration
	{
		int m_LeftSensorCount;
		int m_RightSensorCount;
		int m_TopSensorCount;
		int m_BtmSensorCount;
	};

	RobotBodyOutline( SensorConfiguration sensorconfiguration );
	//
	QRectF boundingRect() const;
	//
	void paint( QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget );
	//
	void CalcSensorLocations( int sensorwidth, SensorLocationList& locationlist );
	//
protected:
	//
	int m_TotalHeight;
	int m_TotalWidth;

	int m_Radius;
	int m_PenWidth;

	int m_LeftMargin;
	int m_BottomMargin;
	int m_TopMargin;
	int m_RightMargin;
	//
	SensorConfiguration m_SensorConfiguration;
	//
private:
	//
};

#endif /* ROBOTBODYOUTLINE_H_ */
