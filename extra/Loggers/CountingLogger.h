#ifndef COUNTINGLOGGER_H
#define COUNTINGLOGGER_H

#include <boost/shared_ptr.hpp>

#include <string>
#include <sstream>

namespace Loggers
{
    class CountingLogger
    {
    public:
        typedef boost::shared_ptr<CountingLogger> Ptr ;
        CountingLogger(const std::string& name) : m_name(name), m_numberOfLogCalls(0) {}
        void Log()
        {
            m_numberOfLogCalls++;
        }

        int NumberOfLogCalls()
        {
            return m_numberOfLogCalls ;
        }

        std::string ToString(const std::string& header)
        {
            std::stringstream out ;

            out << header << "[" << m_name << "]" << " Received " << NumberOfLogCalls() << " Log Message(s)" << std::endl ;

            return out.str();
        }

    private:
        std::string m_name ;
        int m_numberOfLogCalls ;
    };
};

#endif // COUNTINGLOGGER_H
