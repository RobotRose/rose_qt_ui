//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

/*
 * SensorRingController.h
 *
 *  Created on: Sep 23, 2010
 *      Author: martijn
 */

#ifndef SENSORRINGCONTROLLER_H_
#define SENSORRINGCONTROLLER_H_
//---------------------------------------------------------------------------------
//
//
#include "std_msgs/Int8MultiArray.h"
#include "ApplicationServices/ListenerImplementation.h"
//
#include <boost/shared_ptr.hpp>
//
#include "SensorRing/SensorRingDisplay.h"
//---------------------------------------------------------------------------------
//
//
class SensorRingController : public ApplicationServices::ListenerImplementation<const std_msgs::Int8MultiArray::ConstPtr>
{
public:
// 	typedef boost::shared_ptr<SensorRingController> Ptr ;
	//
	SensorRingController( boost::shared_ptr<SensorRingDisplay> display );
	//
protected:
	//
	void CallbackImplementation( const std_msgs::Int8MultiArray::ConstPtr& param );
	//
private:
	//
	boost::shared_ptr<SensorRingDisplay> m_Object;
	//
};
//---------------------------------------------------------------------------------
//
//
#endif /* SENSORRINGCONTROLLER_H_ */
