//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

/*
 * CamController.h
 *
 *  Created on: Oct 1, 2010
 *      Author: martijn
 */

#ifndef CAMCONTROLLER_H_
#define CAMCONTROLLER_H_
//---------------------------------------------------------------------------------
//
#include <boost/shared_ptr.hpp>
//
#include "sensor_msgs/Image.h"
//---------------------------------------------------------------------------------
//
//
#include "ApplicationServices/ListenerImplementation.h"
#include "InteractiveImageDisplay/WebCamImageDisplay.h"
//---------------------------------------------------------------------------------
//
class CamController : public ApplicationServices::ListenerImplementation<const sensor_msgs::ImageConstPtr>
{
public:
	//
    CamController(WebCamImageDisplay* cameraDisplay );
    ~CamController()
    {
        std::cout << "~CamController:" << std::endl;
    }
	//
protected:
	//
	// The ApplicationServices::ListenerImplementation interface
	void CallbackImplementation(const sensor_msgs::ImageConstPtr& param);
	//
private:
	//
    WebCamImageDisplay* m_CameraDisplay;
	//
};
//---------------------------------------------------------------------------------
//
//
#endif /* CAMCONTROLLER_H_ */
