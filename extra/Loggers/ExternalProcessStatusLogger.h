#ifndef EXTERNALPROCESSSTATUSLOGGER_H
#define EXTERNALPROCESSSTATUSLOGGER_H

#include <boost/shared_ptr.hpp>

#include <string>
#include <sstream>

namespace Loggers
{
    class ExternalProcessStatusLogger
    {
    public:
        typedef enum {OK, NOK} EnumOkNok ;
        typedef boost::shared_ptr<ExternalProcessStatusLogger> Ptr ;
        ExternalProcessStatusLogger(const std::string& name) : m_name(name), m_startedOK(false), m_stoppedOK(false), m_failureDetected(false) {}

        void LogStarted(EnumOkNok status)
        {
            if(OK==status)
            {
                m_startedOK = true;
            }
            else
            {
                m_startedOK = false;
            }
        }

        void LogStopped(EnumOkNok status)
        {
            if(OK==status)
            {
                m_stoppedOK = true;
            }
            else
            {
                m_stoppedOK = false;
            }
        }

        void FailureDetected()
        {
            if(!m_stoppedOK)
            {
                m_failureDetected = true ;
            }
        }

        void Reset()
        {
            m_startedOK = false;
            m_stoppedOK = false;
            m_failureDetected = false;
        }

        std::string ToString(const std::string& header)
        {
            std::stringstream out ;

            out << header << "[" << m_name << "]" << " Received: " << "Started:" << (m_startedOK ? "OK" : "NOK") << " Stopped:" << (m_stoppedOK ? "OK" : "NOK") <<
                   " FailureDetected:" << (m_failureDetected ? "YES" : "NO") << std::endl ;

            return out.str();
        }

    private:
        std::string m_name ;
        bool m_startedOK ;
        bool m_stoppedOK ;
        bool m_failureDetected;
    };
};

#endif // EXTERNALPROCESSSTATUSLOGGER_H
