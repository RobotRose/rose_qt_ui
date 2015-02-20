#ifndef DATABASE_H_
#define DATABASE_H_

// Database.h
// Copyright Sioux B.V. 2010
//
// Description: Interface to the database wrapper. A concrete class will implement this interface.

#include "DatabaseResult.h"

#include <boost/shared_ptr.hpp>

namespace ApplicationServices
{

	class Database
	{
	public:
		virtual ~Database() {}
		virtual boost::shared_ptr<ApplicationServices::DatabaseResult> ExecuteQuery(const std::string& query)=0;
		virtual boost::shared_ptr<ApplicationServices::DatabaseResult> GetNextResult()=0;
	};
};

#endif /* DATABASE_H_ */
