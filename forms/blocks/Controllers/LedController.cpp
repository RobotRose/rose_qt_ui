//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

/*
 * LedController.cpp
 *
 *  Created on: Sep 24, 2010
 *      Author: martijn
 */
//---------------------------------------------------------------------------------
//
//
#include <assert.h>
#include "../Led/Led.h"       //2011/04/05   nv Controller get more and more mixed up with Led
#include "LedController.h"
//---------------------------------------------------------------------------------
//

LedController::LedController( boost::shared_ptr< CLed > led )
    : m_Object(led)
{
    assert( m_Object );
//  if(!m_Object) { m_Object = boost::shared_ptr<BinToggle>(new EmptyBinToggle()); }
}
//---------------------------------------------------------------------------------
//
//
void LedController::CallbackImplementation(const std_msgs::Bool::ConstPtr& param)
{
    m_Object->ConsumeData( param );
}
//---------------------------------------------------------------------------------
//
//
