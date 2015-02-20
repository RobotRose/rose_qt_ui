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
#ifndef VIDEOCAPTURE_H_
#define VIDEOCAPTURE_H_

#include <boost/shared_ptr.hpp>
#include "ApplicationServices/Runnable.h"
#include "Ros/RosWrapper.h"
//
#include "usb_cam.h"
//
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

//
namespace Video
{
    /**
     * Capture video image.
     */
    class VideoCapture : public ApplicationServices::Runnable
    {
    public:
    	//
    	typedef boost::shared_ptr<VideoCapture> Ptr;
    	//
    	VideoCapture( std::string device_name, std::string topic_name );
    	VideoCapture( std::string device_name, std::string topic_name, Ros::RosWrapper::Ptr ros_wrapper );

        void setResolution( int width, int height );
        void setFps( int fps );
        bool isStarted(){ return is_started_; }

        /**
         * @brief Pauses the video stream by simply not sending any more frames. 
         * @details The last frame is black, so your faces isn't frozen. Pauzing is done by checking whether a bool is set or not.
         */
        void pause();

        /**
         * @brief Unpauses the video stream. 
         * @details Resets the bool ser by pause().
         */
        void unpause() {is_pauzed_ = false ; is_first_pause_ = false;}
        bool restart();
        
    	// Runnable interface.....
    	bool prepare();
        bool runOnce();
        void finish();
        //
    protected:
        //
    private:
        bool webcam_restart_;
        bool is_started_;
        bool is_pauzed_;

        /**
         * is_first_pause is set by pause() and reset to false by Runnable::runOnce.
         */
        bool is_first_pause_;
        //
		int width_;
		int height_;
		int fps_;
		int modetectLum_;
		int modetectCount_;
		//
		std::string device_;
		std::string output_topic_;
		//
        usb_cam_camera_image_t* camera_image_;
        Ros::RosWrapper::Ptr ros_wrapper_;

        boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
        image_transport::CameraPublisher image_pub_;

        boost::timed_mutex syncroot;
    };
}


#endif /* VIDEOCAPTURE_H_ */
