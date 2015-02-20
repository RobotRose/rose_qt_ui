//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

/*
 * Led.h
 *
 *  Created on: Sep 24, 2010
 *      Author: martijn
 */
//---------------------------------------------------------------------------------
//
//
#ifndef LED_H
#define LED_H
//
#include <QtGui/QWidget>
#include <QtCore/QVariant>

#include "std_msgs/Bool.h"

#include <boost/thread/mutex.hpp>

#include "Loggers/CountingLogger.h"

//
  class CLed : public QWidget 
  {
    Q_OBJECT

  public:

    CLed( QWidget *parent = 0 );
    ~CLed();
    //
    virtual void ConsumeData(const std_msgs::Bool::ConstPtr& param);
    enum eLedState { ledoff, ledon };   //2011/04/18     nv get mock going
    //
  protected:
    void paintEvent( QPaintEvent *event );
    void resizeEvent( QResizeEvent *event );

  private:
    boost::mutex m_cled_mutex ;
    unsigned int m_cled_mutex_cnt ;     //2011/04/14      nv remove me after debug

    void ReConfigureImage( int width, int height );
    void ReConfigureRadialGradiant( QColor color );
    void ReConfigure( QColor color );

    //
//     enum eLedState { ledoff, ledon };  //2011/04/18     nv get mock going
    virtual void setLedState( eLedState state );  
    virtual void customEvent( QEvent *event);

    bool m_state;

    QColor OffColor;
    QColor OnColor;
    QColor CurrentColor;
    QRadialGradient *RadialGradient;
    QImage* Image;
    QPainter* m_Painter;
    boost::shared_ptr<Loggers::CountingLogger> m_logger ;
  };


#endif // CLED_H

