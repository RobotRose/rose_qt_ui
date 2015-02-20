#ifndef DATASTORE_ITEM_CONVERTOR_H
#define DATASTORE_ITEM_CONVERTOR_H

#include <string>
#include <list>

/**
 * Helper class that converts an dataStore item to a string and vice versa.
 */
class DataStoreItemConverter
{
private:
  /**
   * Trim leading and trailing spaces from given string
   */
  static void stringTrimSpaces(std::string& str);

public:
	/**
	*  Convert an integer dataStore item to a string.
	*/
	static std::string itemIntegerToString(int value);

	/**
	*  Convert an string to an integer dataStore item.
	*/
	static int itemStringToInteger(const std::string& value);

	/**
	*  Convert a double dataStore item to a string.
	*/
	static std::string itemDoubleToString(double value);

	/**
	*  Convert an string to an double dataStore item.
	*/
	static double itemStringToDouble(const std::string& value);

	/**
	*  Convert a boolean dataStore item to a string.
	*/
	static std::string itemBooleanToString(bool value);

	/**
	*  Convert an string to an boolean dataStore item.
	*/
	static bool itemStringToBoolean(const std::string& value);

	/**
	*  Convert a list of string to a string dataStore item.
	*/
	static std::string itemListOfStringToString(const std::list<std::string>& list);
	
	/**
	*  Convert a string dataStore item to a list of string.
	*/
	static std::list<std::string> itemStringToListOfString(const std::string& value);
	
	/**
	*  Convert a list of integer to a string dataStore item.
	*/
	static std::string itemListOfIntegerToString(const std::list<int>& list);
	
	/**
	*  Convert a string dataStore item to a list of integer.
	*/
	static std::list<int> itemStringToListOfInteger(const std::string& value);
	
	/**
	*  Convert a list of double to a string dataStore item.
	*/
	static std::string itemListOfDoubleToString(const std::list<double>& list);
	
	/**
	*  Convert a string dataStore item to a list of double.
	*/
	static std::list<double> itemStringToListOfDouble(const std::string& value);
	
	/**
	*  Convert a list of boolean to a string dataStore item.
	*/
	static std::string itemListOfBooleanToString(const std::list<bool>& list);
	
	/**
	*  Convert a string dataStore item to a list of boolean.
	*/
	static std::list<bool> itemStringToListOfBoolean(const std::string& value);
};

#endif /* #ifndef DATASTORE_ITEM_CONVERTOR_H */
