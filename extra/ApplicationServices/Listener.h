
// Listener.cpp
// Copyright Sioux 2010
// Initial Creation date: Sep 23, 2010
//
// Description: Interface to the ROS listener.

#ifndef Listener_H_
#define Listener_H_

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

namespace ApplicationServices
{
	template<typename T>
	class Listener
	{
	public:
		typedef boost::shared_ptr<Listener<T> > Ptr ;

		virtual boost::function< void(T&)> GetBoostFunctionObject() = 0 ;
	};
};

#endif /* Listener_H_ */
