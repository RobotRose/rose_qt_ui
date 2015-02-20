#ifndef PLATFORMJOYSTICKFILELOGGER_H
#define PLATFORMJOYSTICKFILELOGGER_H

#include "Loggers/FileLogger.h"

namespace Loggers
{
    class PlatformJoystickFileLogger : public Logger
    {
    public:
        typedef boost::shared_ptr<PlatformJoystickFileLogger> Ptr ;
        PlatformJoystickFileLogger(Logger::Ptr logger)
            : m_logger(logger), m_state(Stopped)
        {

        }
        void Log(const std::string& logMessage)
        {
            if(Stopped==m_state)
            {
                if("START"==logMessage)
                {
                    m_logger->Log("PlatformJoystic started successfully.");
                    m_state = Started ;
                }
            } else if(Started==m_state)
            {
                if("STOP"==logMessage)
                {
                    m_logger->Log("PlatformJoystic stopped successfully.");
                    m_state = Stopped ;
                } else if("FAIL"==logMessage)
                {
                    m_logger->Log("PlatformJoystic failed.");
                    m_state = Stopped ;
                }
            }

        }

    private:
        typedef enum{Started,Stopped} States ;
        Logger::Ptr m_logger ;
        States m_state ;
    };
}

#endif // PLATFORMJOYSTICKFILELOGGER_H
