#ifndef CUSTOMERS_H_
#define CUSTOMERS_H_


// Customers.h
// Copyright Sioux 2010
// Initial Creation date: Sep 10, 2010
//
// Description: Header file for the Customers.cpp

#include "Customer.h"
#include "Database.h"

#include <boost/shared_ptr.hpp>
#include <vector>

namespace ApplicationServices
{

	class Customers
	{
	public:
	    typedef boost::shared_ptr<ApplicationServices::Customers> Ptr ;

		typedef std::vector<ApplicationServices::Customer>::const_iterator const_iterator;
		typedef std::vector<ApplicationServices::Customer>::iterator iterator;
		Customers(boost::shared_ptr<ApplicationServices::Database> db) : m_db(db) {}
		iterator begin() { return m_customers.begin(); }
		iterator end() { return m_customers.end(); }
		const ApplicationServices::Customer& operator[](int index) const {return m_customers[index];}
		int GetNumberOfCustomers() const {return m_customers.size();}

		void Discover() ;

	private:
		boost::shared_ptr<ApplicationServices::Database> m_db ;
		std::vector<ApplicationServices::Customer> m_customers ;
	};
};

#endif /* CUSTOMERS_H_ */
