//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

/*
 * SensorRingDisplay.h
 *
 *  Created on: Sep 24, 2010
 *      Author: martijn
 */
//---------------------------------------------------------------------------------
//
//

#ifndef SENSORRINGDISPLAY_H
#define SENSORRINGDISPLAY_H
//
#include <QtCore/QEvent>
#include <QtGui/QWidget>
#include <QtGui/QGraphicsView>

#include "std_msgs/Int8MultiArray.h"
#include <boost/thread/mutex.hpp>

#include <vector>

#include "RobotBodyOutline.h"

#include "Loggers/UniformVectorLogger.h"

class SensorLed; // let's use a forward declaration i.s.o #include "SensorLed.h"  

//
  class SensorRingDisplay : public QGraphicsView 
  {
    //
    Q_OBJECT
    //
  public:
    //
    struct SensorRingDisplayConfiguration
    {
        int m_SensorCountLeft;
        int m_SensorCountRight;
        int m_SensorCountFront;
        int m_SensorCountRear;
        //
        int m_MinRange_cm;
        int m_MaxRange_cm;
    };
    //
    typedef boost::shared_ptr<SensorRingDisplay> Ptr;
    //
    SensorRingDisplay( QWidget *parent , SensorRingDisplay::SensorRingDisplayConfiguration config  );
    ~SensorRingDisplay();
    //
    void Initialise();
    //
    virtual void ConsumeData( const std_msgs::Int8MultiArray::ConstPtr& param );
  protected:
    //
  private:
    //
    boost::mutex m_mutex_sensorring;
    unsigned int m_mutex_sensorring_cnt;
    void RePositionRobotBodyOutline();
    void RePositionLeds();

	QGraphicsScene m_Scene;
	RobotBodyOutline* m_RobotBodyOutline;
	std::vector<SensorLed*> m_LedList;
	boost::shared_ptr<Loggers::UniformVectorLogger> m_logger ;
    virtual void customEvent( QEvent *event);
};

#endif // SENSORRINGDISPLAY_H

