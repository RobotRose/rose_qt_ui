//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

/*
 * GuGu
 * LabelController.h
 *
 *  Created on: Mar 15, 2010
 *      Author: gurcan
 */

#ifndef LABELCONTROLLER_H_
#define LABELCONTROLLER_H_
//---------------------------------------------------------------------------------
//
//
#include "std_msgs/String.h"
//---------------------------------------------------------------------------------
//
//
#include <boost/shared_ptr.hpp>
#include "ApplicationServices/ListenerImplementation.h"

#include "CLabel/CLabel.h"
//---------------------------------------------------------------------------------
//
//
class LabelController : public ApplicationServices::ListenerImplementation<const std_msgs::String::ConstPtr>
{
    public:
        typedef boost::shared_ptr<LabelController> Ptr ;
        // It get's the label to operate on, and the warning listener as dependency.
        LabelController( boost::shared_ptr<CLabel> label );

        //ToDo this is a pure debug facilty, delete these 2 counts som day in the future (2011/04/07)
        //
    protected:
        // The ApplicationServices::ListenerImplementation interface
        void CallbackImplementation( const std_msgs::String::ConstPtr& param );
        //
    private:
        //
        boost::shared_ptr<CLabel> m_Object;
        //
};
//---------------------------------------------------------------------------------
//
//
#endif /* LABELCONTROLLER_H_ */
