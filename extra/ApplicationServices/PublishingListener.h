#ifndef PUBLISHINGLISTENER_H_
#define PUBLISHINGLISTENER_H_


// PublishingListener.h
// Copyright Sioux 2010
// Initial Creation date: Oct 18, 2010
//
// Description: publishes what it hears

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include "ApplicationServices/Listener.h"
#include "ApplicationServices/Publisher.h"

#include "ApplicationServices/ListenerImplementation.h"
#include "ApplicationServices/PublisherImplementation.h"

#include "Ros/RosWrapper.h"

namespace ApplicationServices
{
    template<typename T, typename RosWrapperClass=Ros::RosWrapper>
	class PublishingListener : public ApplicationServices::ListenerImplementation<T>,
							   public ApplicationServices::PublisherImplementation<T>
	{
        private:
            typedef boost::shared_ptr<RosWrapperClass> RosWrapperClassPtr ;
	public:
            PublishingListener() {}
            PublishingListener(const std::string& topicName, RosWrapperClassPtr rosWrapper)
                : m_topicName(topicName), m_rosWrapper(rosWrapper)
            {

            }

            ~PublishingListener()
            {
                std::cout << "~PublishingListener:" << m_topicName << std::endl;
            }

            typedef boost::shared_ptr<PublishingListener<T,RosWrapperClass> > Ptr ;
            void CallbackImplementation(T& param)
            {
                Notify(param);
            }

            void Subscribe()
            {
                m_rosWrapper->SubscribeToTopic(m_topicName,this->GetBoostFunctionObject());
            }

            void Unsubscribe()
            {
                RegisterObserver(typename ApplicationServices::EmptyListener<T>::Ptr(new ApplicationServices::EmptyListener<T>()));
                m_rosWrapper->UnsubscribeFromTopic(m_topicName);                
            }

        private:
            std::string m_topicName ;
            RosWrapperClassPtr m_rosWrapper ;
	};
};

#endif /* PUBLISHINGLISTENER_H_ */
