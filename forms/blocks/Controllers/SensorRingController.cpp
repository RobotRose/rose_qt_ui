//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

/*
 * SensorRingController.cpp
 *
 *  Created on: Sep 24, 2010
 *      Author: martijn
 */
//---------------------------------------------------------------------------------
//
//
#include "SensorRingController.h"
//---------------------------------------------------------------------------------
//

SensorRingController::SensorRingController( boost::shared_ptr<SensorRingDisplay> display )
    : m_Object( display )
{
    assert( m_Object );
    //ToDo     2011/04/13    nv verify whether assert is sufficient
//     if(!m_Object) { m_Object = boost::shared_ptr<SensorRingDisplayDevice>(new EmptySensorRingDisplayDevice()); }
}
//---------------------------------------------------------------------------------
//
//
void SensorRingController::CallbackImplementation( const std_msgs::Int8MultiArray::ConstPtr& param )
{
    m_Object->ConsumeData( param );
}
//---------------------------------------------------------------------------------
//
//
