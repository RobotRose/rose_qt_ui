#include "ApplicationServices/TimeParamParser.h"

#include <sstream>

ApplicationServices::TimeParamParser::TimeParamParser(const std::string& timeParam) : m_timeToRunInSec(UNLIMITED), m_unit(SEC)
{
    ParseTimeString(timeParam) ;
}

void ApplicationServices::TimeParamParser::ExtractTimeUnit(const std::string& timeStr)
{
    if(std::string::npos != timeStr.find_first_of("smh"))
    {
        if("m"==timeStr.substr(timeStr.find_first_of("smh"),1))
        {
            m_unit = MIN ;
        }
        else if("h"==timeStr.substr(timeStr.find_first_of("smh"),1))
        {
            m_unit = HOUR ;
        }
    }
}

void ApplicationServices::TimeParamParser::ToInt(const std::string& timeStr)
{
    std::stringstream ss(timeStr);

    ss >> m_timeToRunInSec ;
}

void ApplicationServices::TimeParamParser::AdjustTime()
{
     switch(m_unit)
     {
     case SEC:
         break;
     case MIN:
         m_timeToRunInSec *= 60 ;
         break;
     case HOUR:
         m_timeToRunInSec *= 3600 ;
         break;
     }
}

void ApplicationServices::TimeParamParser::ParseTimeString(const std::string& timeStr)
{
    ExtractTimeUnit(timeStr);
    ToInt(timeStr);
    AdjustTime();
}

