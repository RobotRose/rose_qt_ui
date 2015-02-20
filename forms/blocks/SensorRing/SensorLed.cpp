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
#include "SensorLed.h"
//---------------------------------------------------------------------------------
//
//
#include <iostream>
//---------------------------------------------------------------------------------
//
//
#include <QtGui/QPainter>
#include <QtGui/QCursor>
//---------------------------------------------------------------------------------
//
//
SensorLed::SensorLed( int id, int minrange, int maxrange, QColor mincolor, QColor maxcolor )
	:QGraphicsObject()
{
	m_Id = id;
	m_MinRange = minrange;
	m_MaxRange = maxrange;
	//
	m_MinRangeColor = mincolor;
	m_MaxRangeColor = maxcolor;
	//
	m_Width = 9;
	m_Height = 9;
	//
	setAcceptHoverEvents(true);
}
//---------------------------------------------------------------------------------
//
//
void SensorLed::hoverEnterEvent( QGraphicsSceneHoverEvent * event )
{
	std::cout << "Led id : " << m_Id << std::endl;
}
//---------------------------------------------------------------------------------
//
//
QRectF SensorLed::boundingRect() const
{
	return QRectF(0, 0, m_Width, m_Height );
}
//---------------------------------------------------------------------------------
//
//
void SensorLed::paint( QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget )
{
	painter->setRenderHint(QPainter::Antialiasing);
	QPen pen = painter->pen();
	pen.setColor( Qt::black );
	pen.setWidth( 2 );
	painter->setPen( pen );
	QBrush brush( m_CurrentColor );
	painter->setBrush( brush );
	painter->fillRect( 0, 0, m_Width, m_Height, brush );
	painter->drawRect( 0, 0, m_Width, m_Height );
}
//---------------------------------------------------------------------------------
//
//
void SensorLed::SetValue( int value )
{
	// Trunc for programmer/config fuckup.
	if( value < m_MinRange )
		value = m_MinRange;
	if( value > m_MaxRange )
		value = m_MaxRange;
	//
    double rel_progress =  (double) (value - m_MinRange) / (double) ( m_MaxRange - m_MinRange );
	//
	int red = (int)(( m_MaxRangeColor.red() - m_MinRangeColor.red() ) * rel_progress ) + m_MinRangeColor.red();
	int green = (int)(( m_MaxRangeColor.green() - m_MinRangeColor.green() ) * rel_progress ) + m_MinRangeColor.green();
	int blue = (int)(( m_MaxRangeColor.blue() - m_MinRangeColor.blue() ) * rel_progress ) + m_MinRangeColor.blue();
	//
	m_CurrentColor.setRgb( red, green, blue );
}
