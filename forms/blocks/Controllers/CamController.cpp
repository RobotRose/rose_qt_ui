//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

/*
 * CamController.cpp
 *
 *  Created on: Oct 1, 2010
 *      Author: martijn
 */
//---------------------------------------------------------------------------------
//
//#include <iostream>
//#include <string>
//
#include "CamController.h"
//---------------------------------------------------------------------------------
//
//
CamController::CamController(WebCamImageDisplay* cameraDisplay )
    : m_CameraDisplay(cameraDisplay)
{
}
//---------------------------------------------------------------------------------
//
//
void CamController::CallbackImplementation(const sensor_msgs::ImageConstPtr& param)
{
	// pass image to the m_CameraDisplay
    if( not param->data.empty())
      m_CameraDisplay->ShowWebCamImage( param->data, param->width, param->height );
}
//---------------------------------------------------------------------------------
//
//
