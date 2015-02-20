//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

/*
 * BatteryController.h
 *
 *  Created on: Sep 23, 2010
 *      Author: martijn
 */

#ifndef BATTERYCONTROLLER_H_
#define BATTERYCONTROLLER_H_
//---------------------------------------------------------------------------------
//
//
#include "rose_base_msgs/battery_state.h"
//---------------------------------------------------------------------------------
//
//
#include <boost/shared_ptr.hpp>
#include "ApplicationServices/ListenerImplementation.h"
//---------------------------------------------------------------------------------
//
//
class BatteryController : public ApplicationServices::ListenerImplementation<const rose_base_msgs::battery_state::ConstPtr>
{
public:
// 	typedef boost::shared_ptr<BatteryController> Ptr ;
	//
	BatteryController( boost::shared_ptr<Battery> battery );
	//
protected:
	//
	void CallbackImplementation( const rose_base_msgs::battery_state::ConstPtr& param );
	//
private:
	//
	boost::shared_ptr<Battery> m_Object;
	//
};
//---------------------------------------------------------------------------------
//
//
#endif /* BATTERYCONTROLLER_H_ */
