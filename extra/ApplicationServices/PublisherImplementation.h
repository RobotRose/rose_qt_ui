#ifndef PUBLISHERIMPLEMENTATION_H_
#define PUBLISHERIMPLEMENTATION_H_

// PublisherImplementation.h
// Copyright Sioux 2010
// Initial Creation date: Oct 18, 2010
//
// Description: PublisherImplementation.

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include "ApplicationServices/Publisher.h"
#include "ApplicationServices/Listener.h"
#include "ApplicationServices/EmptyListener.h"

namespace ApplicationServices
{
    template<typename T>
    class PublisherImplementation : public ApplicationServices::Publisher<T>
    {
    public:
    	PublisherImplementation()
              : m_listener(boost::shared_ptr<ApplicationServices::EmptyListener<T> >(new ApplicationServices::EmptyListener<T>()))
        {
        }
        ~PublisherImplementation()
        {
        }
        void RegisterObserver(boost::shared_ptr<ApplicationServices::Listener<T> > listener)
        {
            m_listener = listener;
        }

        void Notify(T& param)
        {
        	m_listener->GetBoostFunctionObject()(param) ;
        }

    protected:
        boost::shared_ptr<ApplicationServices::Listener<T> > m_listener;

    };

};


#endif /* PUBLISHERIMPLEMENTATION_H_ */
