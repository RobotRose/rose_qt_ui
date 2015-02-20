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
#ifndef INTERACTIONOBJECT_HPP
#define INTERACTIONOBJECT_HPP
//
#include "rose_main/interactive_image_display/overlay/overlay_settings.hpp"
//
class InteractionObject
{
	//
public:
	//
	virtual void MoveStepUp() = 0;
	virtual void MoveUpStart() = 0;
	virtual void MoveUpStop() = 0;
	//
	virtual void MoveStepDown() = 0;
	virtual void MoveDownStart() = 0;
	virtual void MoveDownStop() = 0;
	//
	virtual void MoveStepLeft() = 0;
	virtual void MoveLeftStart() = 0;
	virtual void MoveLeftStop() = 0;
	//
	virtual void MoveStepRight() = 0;
	virtual void MoveRightStart() = 0;
	virtual void MoveRightStop() = 0;
	//
	virtual void MoveHome() = 0;
	//
	virtual void SelectCam( LRCameraSelector cam ) = 0;
};


#endif /* INTERACTIONOBJECT_HPP */
