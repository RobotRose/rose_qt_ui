//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

/*
 * BatteryController.cpp
 *
 *  Created on: Sep 24, 2010
 *      Author: martijn
 */
//---------------------------------------------------------------------------------
//
//
#include <assert.h>
#include "Battery/Battery.h"
#include "BatteryController.h"
//---------------------------------------------------------------------------------
//

BatteryController::BatteryController( boost::shared_ptr< Battery > battery )
    : m_Object(battery)
{
    assert( m_Object );
}
//---------------------------------------------------------------------------------
//
//
void BatteryController::CallbackImplementation(const rose_base_msgs::battery_state::ConstPtr& param)
{
    m_Object->ConsumeData( param );
}
//---------------------------------------------------------------------------------
//
//
