//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================


#ifndef TSRSLIDER_H
#define TSRSLIDER_H
//
#include <QtGui/QWidget>
#include <QtCore/QEvent>
#include <QtCore/QVariant>
#include <QtCore/QTimer>

#include <iostream>
#include <ros/ros.h>
#include "std_msgs/Int32.h"

#include <boost/thread/mutex.hpp>

  class SliderSlave
  {
  public:
	  typedef boost::shared_ptr<SliderSlave> Ptr;

      /**
       *  NewSetpointAvailable is called whenever the slider position is changed.
       *  @param value the position of the slider
       *  @param dragging True when the value is being dragged and false when not.
       */
      virtual void NewSetpointAvailable( int value) = 0;
      virtual void ChangeStarted() = 0;
      virtual void ChangeFinished() = 0;
  };

  class EmptySliderSlave : public SliderSlave
  {
  public:
	  typedef boost::shared_ptr<EmptySliderSlave> Ptr;
	  EmptySliderSlave( std::string name )
	  :m_Name( name )
	  {

	  }
      virtual void NewSetpointAvailable( int value)
	  {
		  std::cout << "SliderSlave " << m_Name << " new setpoint : " << value << std::endl;
	  };

      virtual void ChangeStarted()
      {
      }

      virtual void ChangeFinished()
      {
      }
          ;
  private:
	  std::string m_Name;
  };

  class PublishingSliderSlave : public SliderSlave
  {
  public:
      typedef boost::shared_ptr<PublishingSliderSlave> Ptr;
      PublishingSliderSlave( std::string name, std::string topic )
      :m_Name( name )
      , m_CurrentSetpoint_(0)
      {
          ros::NodeHandle n;
          m_Publisher_ = n.advertise<std_msgs::Int32>(topic, 1);
      }

      virtual void NewSetpointAvailable( int value)
      {
          ROS_INFO("New setpoint for slider %s: %i", m_Name.c_str(), value);
          m_CurrentSetpoint_ = value;
      };

      virtual void ChangeStarted()
      {
      }

      virtual void ChangeFinished()
      {
          ROS_INFO("Change is finished, sending width %i", m_CurrentSetpoint_);
          std_msgs::Int32 msg;
          msg.data = m_CurrentSetpoint_;
          m_Publisher_.publish(msg);
      }

  private:
      std::string m_Name;
      ros::Publisher m_Publisher_;
      int m_CurrentSetpoint_;
  };
//
  class Slider
  {
  public:
	  typedef boost::shared_ptr<Slider> Ptr;
	  enum SliderMode { smDirectFollow, smWillReiveUpdates };
	  virtual void SetCurrentValue( int value ) = 0;
	  virtual void SetSliderMode( Slider::SliderMode mode ) = 0;
	  virtual void SetTickEnable( bool enable ) = 0;
  };

  class TSRSlider : public QWidget, public Slider
  {
    Q_OBJECT
    
  public:
    //
	typedef boost::shared_ptr<TSRSlider> Ptr;
	//
    TSRSlider( QWidget *parent, SliderSlave::Ptr sliderslave );
    ~TSRSlider();
    //
    virtual void SetCurrentValue( int value );
    virtual void SetSliderMode( SliderMode mode );
    virtual void SetTickEnable( bool enable );
    //
  protected:
    //
    virtual void paintEvent( QPaintEvent *event );
    virtual void resizeEvent( QResizeEvent *event );
    virtual void mouseMoveEvent ( QMouseEvent * event );
    virtual void mousePressEvent ( QMouseEvent * event );
    virtual void mouseReleaseEvent ( QMouseEvent * event );
    //
  private:
    //
    void ReCalcConstants();
    int CalcMidSliderPosFromUserValue( int value );
    int CalcUserValueFromMidSliderPosition( int midsliderposition );
    void UpdateSliderRectWithMidSliderPos( int midsliderpos );
    void DrawTicks();
    void ReConfigureImage( int width, int height );
    void RedrawOnImage();
    void DrawSlider();
    void DrawProgress();
    void DrawSliderRectangle();
    void FillSliderRectangle();
    //
    SliderSlave::Ptr m_SliderSlave;
    Slider::SliderMode m_SliderMode;
    //
	int m_LeftMargin;
	int m_rightMargin;
	int m_TopMargin;
	int m_BtmMargin;

	int m_WorkingWidth;
	int m_WorkingHeight;
	int m_Radius;
//
	int m_MinPos;
	int m_MaxPos;

	int m_OutherLineWidth;
	QColor m_SliderColor;
	QColor m_OutherLineColor;
    int m_SliderWidth;
	QRect m_SliderRect;
	QRect m_MainRect;
	QRect m_ProgressRect;

	bool m_HooveredOver;
	bool m_DraggingSlider;
	bool m_TicksEnable;

	int m_MaximumValue;
	int m_MinimumValue;
	int m_CurrentValue;
	int m_UserSetpoint;

	boost::mutex m_mutex ;
    QImage* Image;
    QPainter* m_Painter;
  };



#endif // TSRSLIDER_H

