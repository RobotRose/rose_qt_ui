#ifndef PUBLISHER_H_
#define PUBLISHER_H_


// Publisher.h
// Copyright Sioux 2010
// Initial Creation date: Oct 18, 2010
//
// Description: publisher interface

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include "ApplicationServices/Listener.h"

namespace ApplicationServices
{
    template<typename T>
    class Publisher
    {
    public:
        typedef boost::shared_ptr<Publisher> Ptr ;
        virtual void RegisterObserver(boost::shared_ptr<ApplicationServices::Listener<T> > listener) = 0;
        virtual void Notify(T&) = 0;
    };

};


#endif /* PUBLISHER_H_ */
