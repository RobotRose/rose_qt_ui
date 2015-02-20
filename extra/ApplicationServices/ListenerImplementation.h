// ListenerImplementation.h
// Copyright Sioux 2010
// Initial Creation date: Oct 19, 2010
//
// Description: ListenerImplementation.

#ifndef LISTENERIMPLEMENTATION_H_
#define LISTENERIMPLEMENTATION_H_

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>


#include "ApplicationServices/Listener.h"

namespace ApplicationServices
{
    template<typename T>
    class ListenerImplementation : public ApplicationServices::Listener<T>
    {
    public:
    	typedef boost::shared_ptr<ListenerImplementation<T> > Ptr ;
    	ListenerImplementation()
            : m_callback(boost::bind(&ListenerImplementation<T>::CallbackMethod, this,_1))
        {
        }
        ~ListenerImplementation()
        {
        }

        boost::function< void(T&)> GetBoostFunctionObject()
        {
            return m_callback;
        }

    protected:
        virtual void CallbackImplementation(T& param) = 0;

    private:
        void CallbackMethod(T& param)
        {
            CallbackImplementation(param);
        }

        boost::function< void(T&)> m_callback;
    };
};

#endif /* LISTENERIMPLEMENTATION_H_ */
