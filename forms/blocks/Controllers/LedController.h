//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

/*
 * LedController.h
 *
 *  Created on: Sep 23, 2010
 *      Author: martijn
 */

#ifndef LEDCONTROLLER_H_
#define LEDCONTROLLER_H_
//---------------------------------------------------------------------------------
//
//
#include "std_msgs/Bool.h"
#include "ApplicationServices/ListenerImplementation.h"
//
#include <boost/shared_ptr.hpp>
//---------------------------------------------------------------------------------
//
//
class LedController : public ApplicationServices::ListenerImplementation<const std_msgs::Bool::ConstPtr>
{
public:
// 	typedef boost::shared_ptr<LedController> Ptr ;
	// It get's the led to operate on, and the HeartBeatlistener as dependency.
	LedController( boost::shared_ptr<CLed> led );
	//
protected:
	// The ApplicationServices::ListenerImplementation interface
	void CallbackImplementation( const std_msgs::Bool::ConstPtr& param );
	//
private:
	//
	boost::shared_ptr<CLed> m_Object;
	//
};
//---------------------------------------------------------------------------------
//
//
#endif /* LEDCONTROLLER_H_ */
