#ifndef ROSSUBSCRIBER_H_
#define ROSSUBSCRIBER_H_


// RosSubscriber.h
// Copyright Sioux 2010
// Initial Creation date: Oct 18, 2010
//
// Description: subscribes to ros

#include <boost/shared_ptr.hpp>

#include "ros/ros.h"

#include "ApplicationServices/PublishingListener.h"
#include "ApplicationServices/Runnable.h"
#include "Ros/RosWrapper.h"

namespace Ros
{    
    class RosRunner : public ApplicationServices::Runnable
    { 
    public:
        typedef boost::shared_ptr<RosRunner> Ptr;

        RosRunner()
            : m_loopRate(DEFAULT_LOOPRATE)
        {

        }        
        // The Runnable interface
        virtual bool prepare()
        {
            return true ;
        }
        virtual bool runOnce()
        {
            ros::spinOnce();
            m_loopRate.sleep();
            return false;
        }
        virtual void finish()
        {

        }
    private:
        static const int DEFAULT_LOOPRATE = 100;        
        ros::Rate m_loopRate ;
    };

};

#endif /* ROSSUBSCRIBER_H_ */
