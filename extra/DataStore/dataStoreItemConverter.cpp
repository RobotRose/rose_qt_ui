#include <string>
#include <stdio.h>
#include "DataStore/dataStoreItemConverter.h"
#include "DataStore/ClientLogger.h"

static const std::string SEPARATOR(";");

// Split a string into multiple string using the separator string.
static void stringSplit(const std::string& inputStr, std::list<std::string>& results){
    std::string str = inputStr;
    size_t found = str.find_first_of(SEPARATOR);
    while(found != std::string::npos){
        if (found > size_t(0)) {
            results.push_back(str.substr(size_t(0),found));
        }
        str = str.substr(found+1);
        found = str.find_first_of(SEPARATOR);
    }
    if (str.length() > 0) {
        results.push_back(str);
    }
}

/**
 * Helper class that converts an dataStore item to a string and vice versa.
 */
 
/**
*  Convert an integer dataStore item to a string.
*/
/* static */
std::string 
DataStoreItemConverter::itemIntegerToString(int value)
{
	char strValue[128];
	snprintf(strValue, sizeof(strValue), "%d", value);
	return strValue;
}

/**
*  Convert an string to an integer item.
*/
/* static */
int 
DataStoreItemConverter::itemStringToInteger(const std::string& value)
{
	int intValue(0);
	if (sscanf(value.c_str(), "%d", &intValue) != 1) 
	{
        DS_LOG_GEN_TRACE(Logging::ClientLogger::GetDefaultLogger(),
                              "ERROR: Could not convert string \"%s\" to integer", value.c_str());
	}
	return intValue;
}

/**
*  Convert a double item to a string.
*/
/* static */
std::string 
DataStoreItemConverter::itemDoubleToString(double value)
{
	char strValue[128];
	snprintf(strValue, sizeof(strValue), "%.10E", value);
	return strValue;
}

/**
*  Convert an string to an double item.
*/
/* static */
double 
DataStoreItemConverter::itemStringToDouble(const std::string& value)
{
	double doubleValue(0.0);
	
	// doubles should be specified with a dot, not a comma. Do some simple type checking.
	size_t found = value.find_first_of(",");
	if (found != std::string::npos) 
	{
        DS_LOG_GEN_TRACE(Logging::ClientLogger::GetDefaultLogger(),
                              "ERROR: Please specify doubles using \".\", not \",\". (item %s)", value.c_str());
	}
	
	// 'e' notation used for double?
  found = value.find_first_of("e");
  if (found != std::string::npos) 
	{
		if (sscanf(value.c_str(), "%le", &doubleValue) != 1) 
		{
            DS_LOG_GEN_TRACE(Logging::ClientLogger::GetDefaultLogger(),
                                      "ERROR: Could not convert string \"%s\" to double", value.c_str());
		}
	}
	else
	{
		// 'E' notation used for double?
		found = value.find_first_of("E");
	  if (found != std::string::npos)
		{
			if (sscanf(value.c_str(), "%lE", &doubleValue) != 1)
			{
                DS_LOG_GEN_TRACE(Logging::ClientLogger::GetDefaultLogger(),
                                              "ERROR: Could not convert string \"%s\" to double", value.c_str());
			}
		}
		else
		{
			// xxx.yyyy notation used for double (no exponent).
			if (sscanf(value.c_str(), "%lf", &doubleValue) != 1) 
			{
                DS_LOG_GEN_TRACE(Logging::ClientLogger::GetDefaultLogger(),
                                              "ERROR: Could not convert string \"%s\" to double", value.c_str());
			}
		}
	}
	return doubleValue;
}

/**
*  Convert a boolean item to a string.
*/
/* static */
std::string 
DataStoreItemConverter::itemBooleanToString(bool value)
{
	std::string strValue("");
	if (value)
	{
		strValue = "true";
	}
	else
	{
		strValue = "false";
	}
	return strValue;
}

/**
*  Convert an string to an boolean item.
*/
/* static */
bool 
DataStoreItemConverter::itemStringToBoolean(const std::string& value)
{
	bool boolValue(false);

  std::string valueStr(value);
  stringTrimSpaces(valueStr);
	if (valueStr == "true")
	{
		boolValue = true;
	}
	else if (valueStr == "false")
	{
		boolValue = false;
	}
	else 
	{
        DS_LOG_GEN_TRACE(Logging::ClientLogger::GetDefaultLogger(),
        "ERROR: Could not convert string \"%s\" to boolean (use \"true\" or \"false\")",
        value.c_str());
	}
	return boolValue;
}

/**
*  Convert a list of string to a string item.
*/
/* static */
std::string 
DataStoreItemConverter::itemListOfStringToString(const std::list<std::string>& list)
{
	std::string strList("");
	std::list<std::string>::const_iterator iter;
	for (iter = list.begin(); iter != list.end(); iter++)
	{
		std::string strValue = *iter;
		if (strList != "")
		{
			strList += SEPARATOR;
		}
		strList += strValue;
	} // for
	return strList;
}

/**
*  Convert a string item to a list of string.
*/
/* static */
std::list<std::string> 
DataStoreItemConverter::itemStringToListOfString(const std::string& value)
{
	std::list<std::string> strList;
	stringSplit(value, strList);
	return strList;
}

/**
*  Convert a list of integer to a string item.
*/
/* static */
std::string 
DataStoreItemConverter::itemListOfIntegerToString(const std::list<int>& list)
{
	std::string strList("");
	std::list<int>::const_iterator iter;
	for (iter = list.begin(); iter != list.end(); iter++)
	{
		int intValue = *iter;
		if (strList != "")
		{
			strList += SEPARATOR;
		}
		strList += DataStoreItemConverter::itemIntegerToString(intValue);
	} // for
	return strList;
}
	
/**
*  Convert a string item to a list of integer.
*/
/* static */
std::list<int> 
DataStoreItemConverter::itemStringToListOfInteger(const std::string& value)
{
	std::list<int> intList;
	std::list<std::string> strList;
	stringSplit(value, strList);
	
	int intValue(0);
	std::string strValue;
	std::list<std::string>::iterator iter;
	for (iter = strList.begin(); iter != strList.end(); iter++)
	{
		strValue = *iter;
		intValue = DataStoreItemConverter::itemStringToInteger(strValue);
		intList.push_back(intValue);
	} // for
	return intList;
}

/**
*  Convert a list of double to a string item.
*/
/* static */
std::string 
DataStoreItemConverter::itemListOfDoubleToString(const std::list<double>& list)
{
	std::string strList("");
	std::list<double>::const_iterator iter;
	for (iter = list.begin(); iter != list.end(); iter++)
	{
		double doubleValue = *iter;
		if (strList != "")
		{
			strList += SEPARATOR;
		}
		strList += DataStoreItemConverter::itemDoubleToString(doubleValue);
	} // for
	return strList;
}
	
/**
*  Convert a string item to a list of double.
*/
/* static */
std::list<double> 
DataStoreItemConverter::itemStringToListOfDouble(const std::string& value)
{
	std::list<double> doubleList;
	std::list<std::string> strList;
	stringSplit(value, strList);
	
	double doubleValue(0.0);
	std::string strValue;
	std::list<std::string>::iterator iter;
	for (iter = strList.begin(); iter != strList.end(); iter++)
	{
		strValue = *iter;
		doubleValue = DataStoreItemConverter::itemStringToDouble(strValue);
		doubleList.push_back(doubleValue);
	} // for
	return doubleList;
}

/**
*  Convert a list of boolean to a string item.
*/
/* static */
std::string 
DataStoreItemConverter::itemListOfBooleanToString(const std::list<bool>& list)
{
	std::string strList("");
	std::list<bool>::const_iterator iter;
	for (iter = list.begin(); iter != list.end(); iter++)
	{
		bool boolValue = *iter;
		if (strList != "")
		{
			strList += SEPARATOR;
		}
		strList += DataStoreItemConverter::itemBooleanToString(boolValue);
	} // for
	return strList;
}

/**
*  Convert a string item to a list of boolean.
*/
/* static */
std::list<bool> 
DataStoreItemConverter::itemStringToListOfBoolean(const std::string& value)
{
	std::list<bool> boolList;
	std::list<std::string> strList;
	stringSplit(value, strList);
	
	bool boolValue(false);
	std::string strValue;
	std::list<std::string>::iterator iter;
	for (iter = strList.begin(); iter != strList.end(); iter++)
	{
		strValue = *iter;
		boolValue = DataStoreItemConverter::itemStringToBoolean(strValue);
		boolList.push_back(boolValue);
	} // for
	return boolList;
}

/**
 * Trim leading and trailing spaces from given string
 */
void
DataStoreItemConverter::stringTrimSpaces(std::string& str)
{
  std::string whitespaces (" \t\f\v\n\r");
  std::size_t startpos = str.find_first_not_of(whitespaces);
  std::size_t endpos = str.find_last_not_of(whitespaces);

  // Return an empty string if empty or all spaces
  if((std::string::npos == startpos ) || (std::string::npos == endpos))
  {
    str.clear();
  }
  else
  {
    str = str.substr(startpos, endpos-startpos+1);
  }
}
