//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

/*
 * GuGu
 * LabelController.cpp
 *
 *  Created on: Mar 15, 2010
 *      Author: gurcan
 */
//---------------------------------------------------------------------------------
//
//
#include <assert.h>
#include <QtCore/QEvent>
#include <QtGui/QApplication>
#include "LabelController.h"
//---------------------------------------------------------------------------------
//

LabelController::LabelController( boost::shared_ptr< CLabel > label )
        : m_Object(label)
{
    assert( m_Object );
}
//---------------------------------------------------------------------------------
//
//
void LabelController::CallbackImplementation(const std_msgs::String::ConstPtr& param)
{
    m_Object->ConsumeData( param );
//      std::cout <<  "LabelController::CallbackImplementation: " << m_Object->warning_text << std::endl;
}
//---------------------------------------------------------------------------------
//
//
