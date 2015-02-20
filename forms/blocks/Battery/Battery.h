//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

/*
 * Battery.h
 *
 *  Created on: Sep 24, 2010
 *      Author: martijn
 */
//---------------------------------------------------------------------------------
//
//
#ifndef BATTERY_H
#define BATTERY_H
//
#include <QtGui/QWidget>
#include <QtCore/QEvent>
#include <QtCore/QTimer>

#include <boost/thread/mutex.hpp>

#include "rose_base_msgs/battery_state.h"

#include "Loggers/CountingLogger.h"

//
  class Battery : public QWidget
  {
    Q_OBJECT

  public:
    Battery( QWidget *parent = 0 );
    ~Battery();
    //
    virtual void ConsumeData(const rose_base_msgs::battery_state::ConstPtr& param);
    //
  protected:
    void paintEvent( QPaintEvent *event );
    void resizeEvent( QResizeEvent *event );
  private:
    boost::mutex m_mutex_battery ;
    unsigned int m_mutex_battery_cnt ; //2011/04/14      nv remove me after debug

    void ReConfigureImage( int width, int height );
    void RedrawOnImage();

    void DrawText();
    void DrawBattery();
    void DrawBoundingRectangle();
    void DrawSegments();
    void DrawBlinkingSegment();

    virtual void customEvent( QEvent *event);

    QTimer m_BlinkTimer;
    bool m_LastBlinkState;

    int m_Level;
    QImage* Image;
    QPainter* m_Painter;
    boost::shared_ptr<Loggers::CountingLogger> m_logger ;

	int m_PenWidth;
	int m_BatteryWidth;
	int m_Segmentmargin_left_and_right;
	int m_Batterymain_top_offset;
	int m_Batterymain_btm_offset;
	int m_Contactheight;
	int m_Contactwidth;
	int m_Radius;

  private Q_SLOTS:
    void TimeToChangeBlinkingSegment();

  };


#endif // BATTERY_H

