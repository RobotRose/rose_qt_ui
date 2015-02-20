#ifndef CUSTOMER_H_
#define CUSTOMER_H_


// Customer.h
// Copyright Sioux 2010
// Initial Creation date: Sep 10, 2010
//
// Description: Data class for the customer data.

#include <boost/shared_ptr.hpp>

#include <string>
#include <vector>

namespace ApplicationServices
{

	class Customer
	{
	public:
	    typedef boost::shared_ptr<ApplicationServices::Customer> Ptr;

		Customer( int id, std::string name, std::string address, std::vector<unsigned char> picture, std::string ipaddress, std::vector<unsigned char> appartmentmap  )
			: m_Id(id)
	        , m_Name(name)
	        , m_Address(address)
	        , m_Picture(picture)
			, m_IpAddress(ipaddress)
			, m_Map(appartmentmap)
	    {

	    }
		int GetId() const {return m_Id;}
		std::string GetName() const { return m_Name;}
		std::string GetAddress() const { return m_Address;}
		std::vector<unsigned char> GetPicture() const { return m_Picture;}
		std::string GetIpAddress() const { return m_IpAddress;}
		std::vector<unsigned char> GetMap() const { return m_Map;}
	private:
		int m_Id ;
		std::string m_Name ;
		std::string m_Address ;
		std::vector<unsigned char> m_Picture;
		std::string m_IpAddress;
		std::vector<unsigned char> m_Map ;
	};
};

#endif /* CUSTOMER_H_ */
