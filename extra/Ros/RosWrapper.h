/*
 * RosWrapper.h
 *
 *  Created on: Sep 23, 2010
 *      Author: martijn
 */

#ifndef ROSWRAPPER_H_
#define ROSWRAPPER_H_

#include <boost/variant.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"

namespace Ros
{

	class RosWrapper
	{
	public:
	    typedef boost::shared_ptr<RosWrapper> Ptr ;
		RosWrapper()
			: m_currentQueueSize(DEFAULT_QUEUE_SIZE)
		{
		}

		template<typename T>
		void SubscribeToTopic(const std::string& topicName, boost::function< void(T&)> m_callback )
		{
			std::cout << "Subscribing to topic : " << topicName << std::endl;
			m_subscribers[topicName].push_back(RosSubscribers(m_nodeHandle.subscribe(topicName,m_currentQueueSize,m_callback)));
		}

		void UnsubscribeFromTopic(const std::string& topicName)
		{
			std::cout << "Unsubscribing from topic : " << topicName << std::endl;
			m_subscribers.erase(topicName);
		}

		template<typename T>
		void PublishToTopic(const std::string& topicName, const T& dataToBePublished)
		{
			if(!TopicPublished(topicName))
			{
				m_publishers[topicName] = RosPublishers(m_nodeHandle.advertise<T>(topicName,m_currentQueueSize));
				// Wait a bit before the topic is ready for use (networking time)
				sleep(1);
			}
			boost::get<ros::Publisher>(m_publishers[topicName]).publish(dataToBePublished);
		}

		int SetQueueSize(int queueSize)
		{
			int currentQueueSize = m_currentQueueSize ;
			m_currentQueueSize = queueSize ;
			return currentQueueSize ;
		}

		int GetQueueSize() const
		{
			return m_currentQueueSize ;
		}

	private:
		bool TopicPublished(const std::string& topicName)
		{
			return m_publishers.find(topicName)!=m_publishers.end();
		}
		typedef boost::variant<ros::Publisher, image_transport::Publisher> RosPublishers ;
		typedef boost::variant<ros::Subscriber, image_transport::Subscriber> RosSubscribers ;
		const static int DEFAULT_QUEUE_SIZE = 1000 ;
		std::map<std::string,RosPublishers> m_publishers ;
		std::map<std::string,std::list<RosSubscribers> > m_subscribers ;

		ros::NodeHandle m_nodeHandle;
		int m_currentQueueSize;
	};

    template<>
    inline void RosWrapper::SubscribeToTopic<const sensor_msgs::ImageConstPtr>(const std::string& topicName, boost::function< void(const sensor_msgs::ImageConstPtr&)> callBack )
    {
    	//std::cout << "Subscribing to topic : " << topicName << std::endl;
        image_transport::ImageTransport imageTransport(m_nodeHandle);
        m_subscribers[topicName].push_back(RosSubscribers( imageTransport.subscribe(topicName,m_currentQueueSize,callBack)));
    }

	template<>
	inline void RosWrapper::PublishToTopic<sensor_msgs::Image>(const std::string& topicName, const sensor_msgs::Image& dataToBePublished)
	{
		if(!TopicPublished(topicName))
		{
			image_transport::ImageTransport imageTransport(m_nodeHandle);

			m_publishers[topicName] = RosPublishers(imageTransport.advertise(topicName,m_currentQueueSize));
			// Wait a bit before the topic is ready for use (networking time)
			sleep(1);
		}
		boost::get<image_transport::Publisher>( m_publishers[topicName] ).publish( dataToBePublished );
	}
};

#endif /* ROSWRAPPER_H_ */
