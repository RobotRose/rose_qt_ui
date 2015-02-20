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
#ifndef MAPDISPLAYINTERACTIONOBJECT_H
#define MAPDISPLAYINTERACTIONOBJECT_H

#include "rose_main/interactive_image_display/interaction_object.hpp"

#include "UserCockpitConfiguration.h"

namespace UCC = UserCockpitConfiguration;

template <typename T>

class MapDisplayInteractionObject : public InteractionObject
{
public:
	typedef boost::shared_ptr<MapDisplayInteractionObject> Ptr;

	struct MapDisplayInteractionObjectDependencies
	{
		UCC::UserCockpitConfigurationManager::Ptr m_CockpitConfiguration;
		typename T::Ptr m_RosWrapper;
	};

	MapDisplayInteractionObject( MapDisplayInteractionObjectDependencies dependencies )
		: m_Dependencies(dependencies)
	{
	}

	~MapDisplayInteractionObject()
	{
	    std::cout << " MapDisplayInteractionObject::~MapDisplayInteractionObject " << std::endl;
	}

	virtual void MoveStepUp()
	{
	}
	
	virtual void MoveUpStart()
	{
	}
	
	virtual void MoveUpStop()
	{
	}
	
	virtual void MoveStepDown()
	{
	}
	
	virtual void MoveDownStart()
	{
	}
	
	virtual void MoveDownStop()
	{
	}
	
	virtual void MoveStepLeft()
	{
	}
	
	virtual void MoveLeftStart()
	{
	}
	
	virtual void MoveLeftStop()
	{
	}
	
	virtual void MoveStepRight()
	{
	}
	
	virtual void MoveRightStart()
	{
	}
	
	virtual void MoveRightStop()
	{
	}
	
	virtual void MoveHome()
	{
	}
	
	virtual void SelectCam( LRCameraSelector cam )
	{
	}
	
private:
	MapDisplayInteractionObjectDependencies m_Dependencies;
};

#endif // MAPDISPLAYINTERACTIONOBJECT_H
