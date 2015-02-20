//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

/*
 * SensorRingDisplay.cpp
 *
 *  Created on: Sep 24, 2010
 *      Author: martijn
 */
//---------------------------------------------------------------------------------
//
//
#include <QtGui/QColor>
#include <QtGui/QImage>
#include <QtGui/QRadialGradient>
#include <QtGui/QPainter>
// 2011/04/05    nv this seems tobe Qt3 #include <QtCore/QCustomEvent>
#include <QtGui/QApplication>
#include <iostream>
//------------------------------------------------------------------------------
//
//
#include "global.h"
#include "SensorRingDisplay.h"
#include "SensorLed.h"  

#include "Loggers/LogMaster.h"
#include "Loggers/UniformVectorLogger.h"

//---------------------------------------------------------------------------------
//
//
SensorRingDisplay::SensorRingDisplay( QWidget *parent , SensorRingDisplay::SensorRingDisplayConfiguration config  )
    : QGraphicsView( parent ),
      m_logger(Loggers::LogMaster::Instance()->Get<Loggers::UniformVectorLogger>("SensorRing"))
{
	// Setting the backgrount to yellow makes debugging easy.
	// setStyleSheet("background: yellow");

	setStyleSheet("background: transparent");
	setFrameStyle( QFrame::NoFrame );
	setHorizontalScrollBarPolicy( Qt::ScrollBarAlwaysOff );
	setVerticalScrollBarPolicy( Qt::ScrollBarAlwaysOff );
	setScene( &m_Scene );

	// We receive the sensor configuration from higher levels.
	RobotBodyOutline::SensorConfiguration sensorconfig;
	sensorconfig.m_BtmSensorCount = config.m_SensorCountRear;
	sensorconfig.m_TopSensorCount = config.m_SensorCountFront;
	sensorconfig.m_LeftSensorCount = config.m_SensorCountLeft;
	sensorconfig.m_RightSensorCount = config.m_SensorCountRight;

	int sensorcount = config.m_SensorCountRear + config.m_SensorCountFront + config.m_SensorCountLeft + config.m_SensorCountRight;

	// Create the robotoutline and pass it the sensorconfiguration.
	m_RobotBodyOutline = new RobotBodyOutline( sensorconfig );
	m_Scene.addItem( m_RobotBodyOutline );

	// Create the correct number of leds, and add them to the scene.
	for( unsigned int i = 0; i < sensorcount; i++ )
	{
		SensorLed* led = new SensorLed( i, 0, 100, Qt::green, Qt::red );
		m_LedList.push_back( led );
		m_Scene.addItem( led );
	}
}
//---------------------------------------------------------------------------------
//
//
SensorRingDisplay::~SensorRingDisplay()
{

}
//---------------------------------------------------------------------------------
//
//
void SensorRingDisplay::Initialise()
{
	RePositionRobotBodyOutline();
	RePositionLeds();
}
//---------------------------------------------------------------------------------
//
//
void SensorRingDisplay::ConsumeData( const std_msgs::Int8MultiArray::ConstPtr& param )
{
    // We receive the array of sensor values from ROS.
    const std::vector<int8_t> values = param->data;
    m_logger->Log(values) ;
    // Check if we have the same number of leds as we have sensor values...
    // Otherwise we do nothing.
    if( values.size() != m_LedList.size() ) {
        std::cout << "Bailing out since sensor count not equal to led count !" << std::endl;
        return;
    }
    // Pass the sensor values to the leds so they can adjust color.
    if ( m_mutex_sensorring.try_lock() == false ) {
        m_mutex_sensorring_cnt++;
//         std::cout << "Battery::ConsumeData try_lock failed " << m_mutex_sensorring_cnt << " times !!!!!!!!!!!!!!!!!!!!" << std::endl ;
    } else {
        for( unsigned int i = 0; i < values.size(); i++ ) {
            m_LedList[i]->SetValue( values[i] );
        }
        QApplication::postEvent( this, new QEvent( SENSORRING_EVENT ) );
        m_mutex_sensorring.unlock();
    }
}
//---------------------------------------------------------------------------------
//
//
void SensorRingDisplay::RePositionLeds()
{
	// The robotbody outline is made in a specific way.
	// It knows how many leds there are, and only it knows
	// where they should be placed with respect to the outline.
	SensorLocationList locationlist;

	// We tell the outline object to calculate the sensor location
	// and put the results in the locactionlist.
	// We then have a xpos, ypos and rotation value.
	int ledsize = m_LedList[0]->boundingRect().width();
	double halfledsize = (double) ledsize / 2.0;
	m_RobotBodyOutline->CalcSensorLocations( ledsize, locationlist );

	// Move the leds accordingly, and set appropriate rotation.
	// This is where the power of the QGraphicsItem comes in.
	// You can freely rotate these things, without having to
	// adjust your drawing algorithm yourself.
	// So the led drawing code remains the same.

	// We do set the rotation origin in the middle of the led.
	for( unsigned int i = 0; i < m_LedList.size(); i++ )
	{
		m_LedList[i]->setTransformOriginPoint( QPointF( halfledsize, halfledsize ) );
		m_LedList[i]->setRotation( (locationlist[i].m_Angle * 180)/( M_PI ) );
		m_LedList[i]->setPos( QPoint(locationlist[i].m_PosX,locationlist[i].m_PosY)  );
	}
}
//---------------------------------------------------------------------------------
//
//
void SensorRingDisplay::RePositionRobotBodyOutline()
{
	m_RobotBodyOutline->setPos(0,0);
}
//---------------------------------------------------------------------------------
//
//

//---------------------------------------------------------------------------------
//
//
void SensorRingDisplay::customEvent( QEvent *event)
{
	update();
}
