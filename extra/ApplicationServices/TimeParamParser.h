#ifndef TIMEPARAMPARSER_H
#define TIMEPARAMPARSER_H

#include <string>

#include <boost/shared_ptr.hpp>

namespace ApplicationServices
{
    class TimeParamParser
    {
    private:
        enum TimeUnit {SEC, MIN, HOUR};
        int m_timeToRunInSec;
        TimeUnit m_unit ;

        void ExtractTimeUnit(const std::string& timeToRunInSecStr);
        void ToInt(const std::string& timeToRunInSecStr);

        void AdjustTime();

        void ParseTimeString(const std::string& timeToRunInSecStr);

    public:
        typedef boost::shared_ptr<TimeParamParser> Ptr ;
        static const int UNLIMITED = -1 ;

        TimeParamParser(const std::string& timeParam);

        int Time()
        {
            return m_timeToRunInSec ;
        }
    };
};

#endif // TIMEPARAMPARSER_H
