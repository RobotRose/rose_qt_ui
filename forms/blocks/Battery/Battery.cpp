//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

/*
 * Battery.cpp
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
#include "Loggers/LogMaster.h"          //needed for cucumber tests
#include "Loggers/CountingLogger.h"
#include "ros/console.h"                //ROS_WARN etc macros
//
#include "global.h"
#include "Battery.h"
//
//---------------------------------------------------------------------------------
//
//
Battery::Battery(QWidget *parent)
	:QWidget ( parent )
	,Image( NULL )
	,m_Painter( NULL )
	,m_Level( 42 )
	,m_LastBlinkState( false )
	,m_logger(Loggers::LogMaster::Instance()->Get<Loggers::CountingLogger>("Battery"))
{
    m_mutex_battery_cnt = 0;

	m_PenWidth = 4;
	m_BatteryWidth = 36;
	m_Segmentmargin_left_and_right = 2;
	m_Batterymain_top_offset = 20;
	m_Batterymain_btm_offset = 10;
	m_Contactheight = 8;
	m_Contactwidth = 18;
	m_Radius = 5;
	//
	ReConfigureImage( 180, 100 );
	RedrawOnImage();
	//
	m_BlinkTimer.setInterval( 250 );
	connect( &m_BlinkTimer, SIGNAL( timeout () ), this, SLOT(TimeToChangeBlinkingSegment() ) );
	m_BlinkTimer.stop();
}
//------------------------------------------------------------------------------
//
//
Battery::~Battery()
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
}
//------------------------------------------------------------------------------
//
//
void Battery::resizeEvent( QResizeEvent *event )
{
	Q_UNUSED(event);
	// When resized, the led needs to be drawn again, smaller or bigger with
	// same settings (aka. color ).
	ReConfigureImage( width(), height() );
	RedrawOnImage();
}
//------------------------------------------------------------------------------
//
//
void Battery::RedrawOnImage()
{
    if ( m_mutex_battery.try_lock() == false ) {
        m_mutex_battery_cnt++;
//         std::cout << "Battery::RedrawOnImage try_lock failed " << m_mutex_battery_cnt << " times !!!!!!!!!!!!!!!!!!!!" << std::endl ;
    } else {
        // Redraw
        if( !m_Painter ) {
            m_Painter = new QPainter();
        }
        //
        Image->fill( Qt::transparent );
        //
        DrawBoundingRectangle();
        DrawBattery();
        DrawText();
        DrawSegments();
        //
        m_mutex_battery.unlock();
        update();
    }
}
//------------------------------------------------------------------------------
//
void Battery::DrawBoundingRectangle()
{
	//
	m_Painter->begin( Image );
	//
	QPen OldPen = m_Painter->pen();
	QPen pen;
	pen.setColor( Qt::blue );
	//
	m_Painter->setPen( pen );
	//
	m_Painter->setPen( OldPen );
	//
	m_Painter->end();
}
//------------------------------------------------------------------------------
//
void Battery::DrawText()
{
	m_Painter->begin( Image );
	//
	QFont font = m_Painter->font();
	font.setPixelSize(30);
	m_Painter->setFont( font );
	// Bounding rectangle.....
	QString text("");
	text.setNum(m_Level);
	text += " %";
	//
	int text_w = m_Painter->fontMetrics().width( text );
	int text_h = m_Painter->fontMetrics().height();
	//
	int xpos = ( 0.75 * width() ) - (text_w/2);
	int ypos = (height()/2)+(text_h/2);
	//
	m_Painter->drawText(xpos, ypos, text );
	//
	m_Painter->end();
}
//------------------------------------------------------------------------------
//
void Battery::DrawBattery()
{
	int xcenter = width()/4;
	int battery_height = height()-m_Batterymain_top_offset - m_Batterymain_btm_offset;

	m_Painter->begin( Image );
	//
	m_Painter->setRenderHint(QPainter::Antialiasing);
	QPen OldPen = m_Painter->pen();

	QPen pen;
	pen.setWidth( m_PenWidth);
	pen.setColor( Qt::darkGray );

	QBrush brush( Qt::darkGray );
	m_Painter->setPen( pen );

	m_Painter->drawRoundedRect( xcenter - m_BatteryWidth/2, m_Batterymain_top_offset, m_BatteryWidth, battery_height, m_Radius, m_Radius );

	m_Painter->fillRect( xcenter - m_Contactwidth/2, m_Batterymain_top_offset - m_Contactheight, m_Contactwidth, m_Contactheight, brush );
	//
	m_Painter->setPen( OldPen );
	m_Painter->end();

}
//------------------------------------------------------------------------------
//
void Battery::DrawSegments()
{
	m_Painter->begin( Image );
	//
	int xcenter = width()/4;

	int segment_width = m_BatteryWidth - m_PenWidth - 6;
	//
	int segments_x = xcenter - ceil(segment_width/2);

	int battery_height = height() - m_Batterymain_top_offset - m_Batterymain_btm_offset;
	//
	int segments_y_min = m_Batterymain_top_offset + 4;
	int segments_y_max = m_Batterymain_top_offset + battery_height - 6;

	int segment_spacing = 3;
	int segment_height = ( segments_y_max - segments_y_min - ( 4 * segment_spacing) ) / 5;

// Helper lines during developing/testing.
//	// left vertical line.... left alignment of segments...
//	m_Painter->drawLine(segments_x, 0,  segments_x, height() );
//	// right vertical line.... right alignment of segments...
//	m_Painter->drawLine(segments_x + segment_width, 0,  segments_x + segment_width, height() );
//	// top horizontal line,.... top alignment of segments.
//	m_Painter->drawLine( 0, segments_y_min,  width(), segments_y_min );
//	// btm horizontal line,.... btm alignment of segments.
//	m_Painter->drawLine( 0, segments_y_max,  width(), segments_y_max );

	QBrush brush( Qt::darkGray );
	m_Painter->setBrush( brush );

	QRect m_BlinkSegmentRect;
	for( unsigned int i = 0; i < 5; i++ )
	{
		if( i < 4 )
		{
			if( m_Level > (100 - ( (i+1) * ( 100 / 5 ) ) ) )
			{
				QBrush brush( Qt::darkGray );
				m_Painter->setBrush( brush );
				m_Painter->fillRect( segments_x, segments_y_min + ( i * segment_spacing ) + ( i * segment_height), segment_width, segment_height , brush );
			}
		}
		else
		{
			QBrush brush( Qt::darkGray );
			if( m_Level < 10 )
			{
				if( m_LastBlinkState )
					brush.setColor( Qt::red );
				m_LastBlinkState = !m_LastBlinkState;
			}
			m_Painter->setBrush( brush );
			m_Painter->fillRect( segments_x, segments_y_min + ( i * segment_spacing ) + ( i * segment_height), segment_width, segment_height , brush );
		}
	}
	//
	if( m_Level < 10 )
	{
		if( !m_BlinkTimer.isActive () )
		{
			m_LastBlinkState = false;
			m_BlinkTimer.start();
		}
	}
	else
	{
		m_BlinkTimer.stop();
	}
	//
	m_Painter->end();
}
//------------------------------------------------------------------------------
//
void Battery::TimeToChangeBlinkingSegment()
{
	RedrawOnImage();
//	update();  2011/03/30    nv since RedrawOnImage just did an update(), I can't think of any purpose to call update() here
//  next question becomes now TimeToChangeBlinkingSegment is identical to RedrawOnImage so what is its purpose
}
//------------------------------------------------------------------------------
//
/* 2011/04/05    nv ReConfigureImage seems a misleading name, since resized is not really used,
 * ReConfigureImage is only used once at instantiation of Battery
 */
void Battery::ReConfigureImage( int width, int height )
{
     if ( m_mutex_battery.try_lock() == false ) {
         m_mutex_battery_cnt++;
//          std::cout << "Battery::ReConfigureImage try_lock failed " << m_mutex_battery_cnt << " times !!!!!!!!!!!!!!!!!!!!" << std::endl ;
     } else {
        // Clean up previous image.
        if( Image ) {
            delete Image;
        }
        Image = new QImage( width, height, QImage::Format_ARGB32 );
        // Redraw
        if( !m_Painter ) {      //2011/04/07     nv what is the purpose of instantiating these QPainter things
            m_Painter = new QPainter();
        }
        //
        Image->fill( Qt::transparent );
        m_mutex_battery.unlock();
     }
}
//------------------------------------------------------------------------------
//
//
void Battery::paintEvent( QPaintEvent *event )
{
    Q_UNUSED(event);
    if( !Image )        //2011/04/07     nv what is this protecting against, i.e. is this legitimate or is it really a bug
    {
//         std::cout << "Battery : paintEvent and Image == NULL !" << std::endl ;
        return;
    }
    //
    if ( m_mutex_battery.try_lock() == false ) {
        m_mutex_battery_cnt++;
//         std::cout << "Battery::paintEvent try_lock failed " << m_mutex_battery_cnt << " times !!!!!!!!!!!!!!!!!!!!" << std::endl ;
    } else {

        QWidget::paintEvent(event);
        //
#if 1 // ;? what does this do, can I leave it out, how does it realte to the instantiation of QPainter elsewhere in this file
        QPainter painter(this);
        painter.drawImage(0,0,*Image);
#endif
        //
        m_mutex_battery.unlock();
    }
}
//------------------------------------------------------------------------------
//
//
/* ConsumeData is now invoked by BatteryController, which is activated by a ROS message. Therefore ConsumeData should
 * never call update()
 */
void Battery::ConsumeData(const rose_base_msgs::battery_state::ConstPtr& param)
{
    m_logger->Log();
    if ( m_mutex_battery.try_lock() == false ) {
        m_mutex_battery_cnt++;
//         std::cout << "Battery::ConsumeData try_lock failed " << m_mutex_battery_cnt << " times !!!!!!!!!!!!!!!!!!!!" << std::endl ;
    } else {
        m_Level = param->percentage;
        QApplication::postEvent( this, new QEvent( BATTERY_INDICATOR_EVENT ) );
        // 	RedrawOnImage();   //this once seemed to cause deadlock 2011/04/06   is now moved to customEvent
        //	update();  2011/03/30    nv since RedrawOnImage already does an update(), this should not have been
        //	commented out but deleted by the previous programmer
        m_mutex_battery.unlock();
    }
}
//---------------------------------------------------------------------------------
//
//

//---------------------------------------------------------------------------------
//
//
void Battery::customEvent( QEvent *event)
{
	RedrawOnImage();  
}
