//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

/*
 * CamIndicator.cpp
 *
 *  Created on: Oct 25, 2010
 *      Author: freddy
 */
//
//
#include "RobotBodyOutline.h"
//---------------------------------------------------------------------------------
//
//
#include <iostream>
#include <math.h>
//---------------------------------------------------------------------------------
//
//
#include <QtGui/QPainter>
#include <QtGui/QCursor>
//---------------------------------------------------------------------------------
//
//
RobotBodyOutline::RobotBodyOutline( SensorConfiguration sensorconfiguration )
	:QGraphicsObject()
{
	m_SensorConfiguration = sensorconfiguration;

	m_TotalHeight = 150;
	m_TotalWidth = 120;

	m_LeftMargin = 10;
	m_RightMargin = 10;
	m_BottomMargin = 10;
	m_TopMargin = 10;

	m_Radius = (m_TotalWidth - m_LeftMargin - m_RightMargin)/2;

	m_PenWidth = 5;
}
//---------------------------------------------------------------------------------
//
//
QRectF RobotBodyOutline::boundingRect() const
{
	return QRectF(0, 0, 120, 150 );
}
//---------------------------------------------------------------------------------
//
//
void RobotBodyOutline::paint( QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget )
{

	painter->setRenderHint(QPainter::Antialiasing);

	QPen pen;

	pen = painter->pen();
        pen.setColor( Qt::black );
	pen.setWidth( m_PenWidth );
	painter->setPen( pen );

        int y1 = m_TopMargin;
	int y2 = m_TotalHeight - m_BottomMargin;

	int x1 = m_LeftMargin;
	int x2 = m_TotalWidth - m_RightMargin;

	// Left vertical line
        painter->drawLine( x1, y1 , x1, y2 );

	// Right vertical line
	painter->drawLine( x2, y1, x2, y2 );

	// Horizontal line, rear
	painter->drawLine( x1, y2, x2, y2 );

        //Horizontal line, front
        painter->drawLine(x1, y1, x2, y1);


}
//---------------------------------------------------------------------------------
//
//
void RobotBodyOutline::CalcSensorLocations( int sensorwidth, SensorLocationList& locationlist )
{
	locationlist.clear();
	double step;

        int x1 = m_LeftMargin;
        int y1 = m_TopMargin;

        int x2 = m_TotalWidth - m_RightMargin;
        int y2 = m_TotalHeight - m_BottomMargin;

        //The distance from the corner that the first sensor is placed on that row:
        int edgeW = m_TotalWidth / 7;
        int edgeH = m_TotalHeight / 7;

        //The distance between each sensor:
        step = ( x2 - x1 - (2 * edgeW) ) / ( m_SensorConfiguration.m_TopSensorCount - 1 );

	for( unsigned int p = 0; p < m_SensorConfiguration.m_TopSensorCount; p++ )
	{
            int pos_x = (int) ( x1 + edgeW + ( (p) * step ) - (sensorwidth/2) );
            int pos_y = y1 - (sensorwidth/2);
            locationlist.push_back( SensorLocation( pos_x, pos_y , 0 ) );
	}

        step = ( y2 - y1 - (2 * edgeH)) / ( m_SensorConfiguration.m_RightSensorCount - 1 );
	for( unsigned int p = 0; p < m_SensorConfiguration.m_RightSensorCount; p++ )
	{
            int pos_x = x2 - (sensorwidth/2);
            int pos_y = (int) (y1 + edgeH + ( (p) * step ) - (sensorwidth/2));
            locationlist.push_back( SensorLocation( pos_x, pos_y , 0 ) );
	}

        step = ( x2 - x1 - (2 * edgeW) ) / ( m_SensorConfiguration.m_BtmSensorCount - 1);
	for( unsigned int p = 0; p < m_SensorConfiguration.m_BtmSensorCount; p++ )
	{
            int pos_x = (int) ( x2 - edgeW - ( (p) * step ) - (sensorwidth/2) );
            int pos_y = y2 - (sensorwidth/2);
            locationlist.push_back( SensorLocation( pos_x, pos_y , 0 ) );
	}

        step = ( y2 - y1 - (2 * edgeH) ) / ( m_SensorConfiguration.m_LeftSensorCount - 1 );
	for( unsigned int p = 0; p < m_SensorConfiguration.m_LeftSensorCount; p++ )
	{
            int pos_x = x1 - (sensorwidth/2);
            int pos_y = (int) (y2 - edgeH - ( (p) * step ) - (sensorwidth/2));
            locationlist.push_back( SensorLocation( pos_x, pos_y , 0 ) );
	}
}
//---------------------------------------------------------------------------------
//
//

