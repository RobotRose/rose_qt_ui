#ifndef DATABASERESULT_H_
#define DATABASERESULT_H_


// Result.h
// Copyright Sioux 2010
// Initial Creation date: Sep 2, 2010
//
// Description: This is the interface for the result set returned from the database.

#include "Value.h"
#include <string>

namespace ApplicationServices
{

	class DatabaseResult
	{
	public:
		virtual ~DatabaseResult() {}
		virtual Value GetValueForFieldAtIndex(int index) const=0;
		virtual bool IsEmpty() const=0;
	};
};

#endif /* DATABASERESULT_H_ */
