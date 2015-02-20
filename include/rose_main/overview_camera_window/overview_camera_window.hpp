/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/03/04
* 		- File created.
*
* Description:
*	description
* 
***********************************************************************************/
#ifndef OVERVIEW_CAMERA_WINDOW_HPP
#define OVERVIEW_CAMERA_WINDOW_HPP

#include <ros/ros.h>
#include <QtGui/QWidget>
 
class OverviewCameraWindow : public QWidget
{
  public:
	OverviewCameraWindow();
	~OverviewCameraWindow();

  private:
  	ros::NodeHandle         n_;
    ros::Publisher          bounding_box_selected_pub_;

    ros::Subscriber         camera_image_sub_;
};

#endif // OVERVIEW_CAMERA_WINDOW_HPP
