// Customers.cpp
// Copyright Sioux 2010
// Initial Creation date: Sep 10, 2010
//
// Description: This is the Application Services layer class for holding
// the Customers registred in the database.

#include "Customers.h"

#include <boost/shared_ptr.hpp>

void ApplicationServices::Customers::Discover()
{
	boost::shared_ptr<DatabaseResult> result = m_db->ExecuteQuery("SELECT * FROM Customers");
	while(!result->IsEmpty())
	{
		m_customers.push_back( ApplicationServices::Customer(   result->GetValueForFieldAtIndex(0),
																result->GetValueForFieldAtIndex(1),
																result->GetValueForFieldAtIndex(2),
																result->GetValueForFieldAtIndex(3),
																result->GetValueForFieldAtIndex(4),
																result->GetValueForFieldAtIndex(5)     ) 	);
		result = m_db->GetNextResult();
	}
}
