/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*    Author: Mathijs de Langen
*    Date  : 2013/12/03
*         - File created.
*
* Description:
*    Class to capture video image.
* 
***********************************************************************************/
#include "VideoCapture.h"
using namespace Video;
//---------------------------------------------------------------------------------
#include "usb_cam.h"
//---------------------------------------------------------------------------------
#include <cstdio>
//---------------------------------------------------------------------------------
#include <ros/ros.h>
#include <ros/time.h>
//---------------------------------------------------------------------------------
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <sensor_msgs/fill_image.h>
//---------------------------------------------------------------------------------
#include <image_transport/image_transport.h>
//---------------------------------------------------------------------------------
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
//---------------------------------------------------------------------------------
#include <cv_bridge/cv_bridge.h>
//---------------------------------------------------------------------------------
#include <camera_info_manager/camera_info_manager.h>
//---------------------------------------------------------------------------------
//
namespace enc = sensor_msgs::image_encodings;

VideoCapture::VideoCapture( std::string device_name, std::string topic_name, Ros::RosWrapper::Ptr ros_wrapper )
	: ros_wrapper_( ros_wrapper )
	, device_( device_name )
	, output_topic_( topic_name )
	, webcam_restart_ ( false )
	, is_started_ ( false )
    , is_pauzed_ ( false )
    , is_first_pause_ ( false )
	, width_ ( 640 )
	, height_ ( 480 )
	, fps_ ( 10 )
{
	ros::NodeHandle n_private;

	n_private.param<std::string>("device", device_, device_ );
	n_private.param<std::string>("topic", output_topic_, output_topic_ );
	
	n_private.param("width", width_, width_);
	n_private.param("height", height_, height_);
	n_private.param("fps", fps_, fps_);

	n_private.param("motion_threshold_luminance", modetectLum_, 100);
	n_private.param("motion_threshold_count", modetectCount_, -1);

	image_transport::ImageTransport it(n_private);
	image_pub_ = it.advertiseCamera(output_topic_, 1);		
	cinfo_.reset(new camera_info_manager::CameraInfoManager(n_private, device_, ""));
}
//---------------------------------------------------------------------------------
//
//
void VideoCapture::setResolution( int width, int height )
{
	width_ = width;
	height_ = height;
	webcam_restart_ = true;
	
}
//---------------------------------------------------------------------------------
//
//
void VideoCapture::setFps (int fps )
{
	fps_ = fps;
	webcam_restart_ = true;
}
//---------------------------------------------------------------------------------
//
//
bool VideoCapture::restart()
{
	return webcam_restart_;
}
//---------------------------------------------------------------------------------
//
//
void VideoCapture::pause()
{
    ROS_DEBUG_NAMED(ROS_NAME, "VideoCapture::pauze()");

    is_pauzed_ = true;
    is_first_pause_ = true;
}
//---------------------------------------------------------------------------------
//
//
bool VideoCapture::prepare()
{
	ROS_DEBUG_NAMED(ROS_NAME, "VideoCapture::prepare()");
	webcam_restart_ = false;

	try
	{
		camera_image_ = usb_cam_camera_start( device_.c_str(), IO_METHOD_USERPTR, PIXEL_FORMAT_MJPEG, width_, height_, fps_ );
	}
	catch( const std::invalid_argument& e )
	{
		//std::cout << "Cannot prepare webcam: " << e.what() << std::endl;
		return false;
	}

	is_started_ = true;
	return true;
}
//---------------------------------------------------------------------------------
//
//
bool VideoCapture::runOnce()
{
    if(!is_pauzed_ || is_first_pause_)
    {
        try
        {
            usb_cam_camera_grab_image( camera_image_ );
        }
        catch ( const std::invalid_argument& e)
        {
            std::cout << "Error in thread: " << e.what() << std::endl;
            return true;
        }

        if( camera_image_->is_new == 1 )
        {
            ros::Time time = ros::Time::now();

            syncroot.lock();

            sensor_msgs::Image image;
            fillImage(image, "rgb8", camera_image_->height, camera_image_->width, 3 * camera_image_->width, camera_image_->image);
            image.header.stamp = time;
            sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));

            ci->header.frame_id = image.header.frame_id;
            ci->header.stamp = image.header.stamp;

            // convert ROS images into OpenCV
            cv_bridge::CvImagePtr cv_ptr;
            //cout << "VideoCapture::runOnce: Calling cv_bridge::toCvCopy. Could be a GUI crash culprit\n";
            try
            {
                std::string src_encoding = std::string(image.encoding.c_str());
                if(std::string("rgb8").compare(src_encoding) != 0) //Unequal
                {
                    ROS_ERROR_NAMED(ROS_NAME, "image has encoding %s != rgb8", src_encoding.c_str());
                    return true;
                }

                cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);

                if(is_pauzed_ && is_first_pause_)
                {
                    ROS_DEBUG_NAMED(ROS_NAME, "VideoCapture::runOnce: blackening image");
                    cv::Rect rect;
                    rect.height = cv_ptr->image.rows;
                    rect.width = cv_ptr->image.cols;
                    cv::rectangle(cv_ptr->image, rect, cv::Scalar( 0, 0, 0), CV_FILLED); //Fill the image with black
                    is_first_pause_ = false;
                }

                image_pub_.publish(*cv_ptr->toImageMsg(), *ci);

             }
            catch (...)
            {
                ROS_ERROR_NAMED(ROS_NAME, "Something went wrong in cv_bridge::toCvCopy. Encoding %s", image.encoding.c_str());
                return true;
            }

            syncroot.unlock();
        }
    }
    return false;
}
//---------------------------------------------------------------------------------
//
//
void VideoCapture::finish()
{
    ROS_DEBUG("Videocapture finish ");
    try
    {
    	usb_cam_camera_shutdown();
    }
    catch( const std::invalid_argument& e )
    {
    	std::cout << "Cannot shutdown webcam: " <<  e.what() << std::endl;
    }
}
//---------------------------------------------------------------------------------
//
//
