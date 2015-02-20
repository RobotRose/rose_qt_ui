#ifndef EMPTYLISTENER_H
#define EMPTYLISTENER_H

// EmptyListener.cpp
// Copyright Sioux 2010
// Initial Creation date: Nov 29, 2010
//
// Description: EmptyListener: class used to realize the NullObject pattern for the listeners.

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>


#include "ApplicationServices/ListenerImplementation.h"

namespace ApplicationServices
{
    template<typename T>
    class EmptyListener : public ApplicationServices::ListenerImplementation<T>
    {
    public:
        typedef boost::shared_ptr<EmptyListener<T> > Ptr ;
    private:
        void CallbackImplementation(T& param)
        {
        }
    };
};

#endif // EMPTYLISTENER_H
