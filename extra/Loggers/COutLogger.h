#ifndef COUTLOGGER_H
#define COUTLOGGER_H

#include <iostream>
#include <boost/shared_ptr.hpp>

namespace Loggers
{
    class COutLogger
    {
    public:
        typedef boost::shared_ptr<COutLogger> Ptr ;
        COutLogger(const std::string& name) : m_name(name) {}
        void Log(const std::string& message)
        {
            m_lastLog = message ;
            std::cout << message << std::endl;
        }

        std::string LastLogMessage()
        {
            return m_lastLog;
        }

        std::string ToString(const std::string& header)
        {
            return "";
        }

    private:
        std::string m_name ;
        std::string m_lastLog ;
    };
};

#endif // COUTLOGGER_H
