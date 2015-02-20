/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/01/17
* 		- File created.
*
* Description:
*	description
* 
***********************************************************************************/
#ifndef CAMERADISPLAY_H_
#define CAMERADISPLAY_H_

#include <QtGui/QWidget>
#include "WebCamImageDisplay.h"

#include "rose_main/interactive_image_display/interactive_image_display.hpp"

class CameraDisplay : public InteractiveImageDisplay, public WebCamImageDisplay
{

public:
    CameraDisplay(  unsigned int componentselection, QWidget * parent );

    virtual void ShowWebCamImage( const std::vector<unsigned char>& data, const int width, const int height );
};

#endif /* CAMERADISPLAY_H_ */
