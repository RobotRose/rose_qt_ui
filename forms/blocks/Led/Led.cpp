//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

/*
 * Led.cpp
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
#include "Loggers/LogMaster.h"
#include "Loggers/CountingLogger.h"
#include "ros/console.h"                //ROS_WARN etc macros
//
#include "global.h"
#include "Led.h"
//
//---------------------------------------------------------------------------------
//
//
CLed::CLed(QWidget *parent)
	:QWidget ( parent )
	,OnColor( Qt::green )
	,OffColor( Qt::lightGray )
	,Image( NULL )
	,RadialGradient( NULL )
	,m_Painter( NULL )
	,m_logger(Loggers::LogMaster::Instance()->Get<Loggers::CountingLogger>("HeartBeat"))
{
	m_cled_mutex_cnt = 0;
	setLedState( ledoff );
}
//------------------------------------------------------------------------------
//
//
CLed::~CLed()
{
	if( m_Painter )
	{
		delete m_Painter;
		m_Painter = NULL;
	}
	if( Image )
	{
		delete Image;
		Image = NULL;
	}
	if( RadialGradient )
	{
		delete RadialGradient;
		RadialGradient = NULL;
	}
}
//------------------------------------------------------------------------------
//
//
void CLed::resizeEvent( QResizeEvent *event )
{
	Q_UNUSED(event);
	// When resized, the led needs to be drawn again, smaller or bigger with
	// same settings (aka. color ).
	ReConfigure( CurrentColor );
}
//------------------------------------------------------------------------------
//
//
void CLed::ReConfigure( QColor color )
{
    static unsigned int cnt_tot;        //ToDo debug code, delete me (2011/04/08)

//     return; //2011/04/18     try to get controller test going
    cnt_tot++;
    if ( m_cled_mutex.try_lock() == false ) {
        m_cled_mutex_cnt++;
       ROS_WARN( "CLed::ReConfigure try_lock failed %d out of %d" , m_cled_mutex_cnt, cnt_tot );
    } else {
        CurrentColor = color;
        ReConfigureRadialGradiant( color );
        ReConfigureImage( width(), height() );
        m_cled_mutex.unlock();
//     return; //2011/04/18     try to get controller test going
        update();
    }
}
//------------------------------------------------------------------------------
//
void CLed::ReConfigureImage( int width, int height )
{
    static unsigned int cnt_del;        //ToDo debug code, delete me (2011/04/08)
    static unsigned int cnt_tot;        //ToDo debug code, delete me (2011/04/08)

    cnt_tot++;
//     ROS_INFO_THROTTLE( 60, "CLed::ReConfigureImage: Clean up previous image %d out of %d", cnt_del, cnt_tot );
    ROS_DEBUG( "CLed::ReConfigureImage: Clean up previous image %d out of %d", cnt_del, cnt_tot );
    // Clean up previous image.
    if( Image ) {
        delete Image;
        cnt_del++;
    }
    Image = new QImage( width, height, QImage::Format_ARGB32 );

    // Redraw
    if( !m_Painter )
        m_Painter = new QPainter();
    //
    Image->fill( Qt::transparent );
    m_Painter->begin( Image );
    m_Painter->setPen( Qt::NoPen );
    m_Painter->setBrush( *RadialGradient );
    m_Painter->setRenderHint( QPainter::Antialiasing );
    m_Painter->drawEllipse( QRect( 1, 1, width-2, height-2) );
    m_Painter->end();           // nv 2011/04/04     this causes a call to paintEvent (I guess)
}
//------------------------------------------------------------------------------
//
//
void CLed::paintEvent( QPaintEvent *event )
{
    Q_UNUSED(event);
    static unsigned int cnt_tot;        //ToDo debug code, delete me (2011/04/08)

    cnt_tot++;
    if( !Image )
    {
        static unsigned int cnt;        //ToDo debug code, delete me (2011/04/08)
        ROS_WARN( "CLed::paintEvent and Image == NULL %d times !!!!!!!!!!!" , ++cnt );
        return;
    }
    //
    if ( m_cled_mutex.try_lock() == false ) {
        m_cled_mutex_cnt++;
        ROS_WARN( "CLed::paintEvent try_lock failed %d out of %d" , m_cled_mutex_cnt, cnt_tot );
    } else {
        QPainter painter(this);
        painter.drawImage(0,0,*Image);

        m_cled_mutex.unlock();
    }
}
//------------------------------------------------------------------------------
//
//
void CLed::setLedState( eLedState state )
{
	if( state == ledon )
		ReConfigure( OnColor );
	else
		ReConfigure( OffColor );
}
//---------------------------------------------------------------------------------
//
//
void CLed::ReConfigureRadialGradiant( QColor color )
{
	// Calculate coordinates
	unsigned w = width();
	unsigned h = height();
	double radius;
	if(w > h)
		radius = w;
	else
		radius = h;

	// Delete old gradient
	if( RadialGradient )
		delete RadialGradient;

	// Create new gradient
	RadialGradient = new QRadialGradient( (w/2), (h/2), (radius/2)-1, (w/2)-(w/10), (h/2)-(h/10)  );

	// set center to middle of Image
	RadialGradient->setCenter( (w/2) , (h/2) );

	// radius is (diameter / 2) - 1 for outer line
	RadialGradient->setRadius( (radius/2)-1 );

	RadialGradient->setFocalPoint( (w/2)-(w/10) , (h/2)-(h/10) );

	// Set new color
	QColor LightColor = color.light(125);
	RadialGradient->setColorAt(0.0,LightColor);
	RadialGradient->setColorAt(0.1,color);
	RadialGradient->setColorAt(0.2,color);
	QColor DarkColor = color.dark(200);
	RadialGradient->setColorAt(1.0,DarkColor);
}

//------------------------------------------------------------------------------
//
//
void CLed::ConsumeData(const std_msgs::Bool::ConstPtr& param)
{
    m_logger->Log();
    if ( m_cled_mutex.try_lock() == false ) {
        m_cled_mutex_cnt++;
//         std::cout << "CLED::ConsumeData try_lock failed " << m_cled_mutex_cnt << " times !!!!!!!!!!!!!!!!!!!!" << std::endl ;
    } else {
        m_state = param->data;
        QApplication::postEvent( this, new QEvent( LED_EVENT ) );
        m_cled_mutex.unlock();
    }
}
//---------------------------------------------------------------------------------
//
//
void CLed::customEvent( QEvent *event)
{
    if ( m_state ) {
        ReConfigure( OnColor );
    } else {
        ReConfigure( OffColor );
    }
}
