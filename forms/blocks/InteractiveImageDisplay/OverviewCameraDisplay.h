//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

/*
 * OverviewCameraDisplay.h
 *
 *  Created on: Feb 16, 2011
 *      Author: nico
 */

#ifndef OVERVIEWCAMERADISPLAY_H_
#define OVERVIEWCAMERADISPLAY_H_

#include <ros/ros.h>

#include <mutex>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <QtGui/QMouseEvent>

#include "CameraDisplay.h"
#include "rose_ui_overview_camera/selection.h"
#include "rose_ui_overview_camera/selections.h"

#include "rose_common/common.hpp"

class OverviewCameraDisplay : public CameraDisplay	
{
	//
	public:
		//
		OverviewCameraDisplay(  unsigned int componentselection, QWidget * parent );
		//
	protected:
		virtual void mousePressEvent ( QMouseEvent * event );
        virtual void mouseReleaseEvent ( QMouseEvent * event );
        virtual void mouseMoveEvent( QMouseEvent * event);

		//
	private:
        void drawBoundingBoxes();
        void CB_boundingBoxesReceived( const rose_ui_overview_camera::selections selections );
        void CB_process_gui_message( const std_msgs::String msg);
        void CB_process_gui_action( const std_msgs::String msg);
        void CB_process_gui_warn( const std_msgs::String msg);

        ros::Publisher 	rect_pub;
        ros::Publisher 	point_pub;
        ros::Subscriber bounding_boxes_sub_;

        //! @todo LvB: Fix this hack, Overview camera is directly
        ros::Subscriber gui_message_sub_;
        ros::Subscriber gui_action_sub_;// $ rostopic pub /messages_window/action std_msgs/String "Klik op de klink" --once #to test these
        ros::Subscriber gui_warn_sub_;

        int x1;          //x-coordinate of left mouse button press
        int y1;          //y-coordinate of left mouse button press;
        double x_scale_;
        double y_scale_ ;

		//
        QGraphicsRectItem *selectionRectangle;
        ros::NodeHandle nh; 

        rose_ui_overview_camera::selections stored_selections_;

        std::vector<QGraphicsRectItem*> bounding_boxes_;
        std::mutex mutex;

        QPointF current_cursor_pos_;
};
//

#endif /* OVERVIEWCAMERADISPLAY_H_ */
