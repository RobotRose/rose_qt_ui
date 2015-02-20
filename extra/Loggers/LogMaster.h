#ifndef LOGMASTER_H
#define LOGMASTER_H

#include <boost/shared_ptr.hpp>
#include <boost/variant.hpp>
#include <boost/foreach.hpp>

#include <string>
#include <map>

#include "Loggers/COutLogger.h"
#include "Loggers/CountingLogger.h"
#include "Loggers/UniformVectorLogger.h"
#include "Loggers/ExternalProcessStatusLogger.h"

namespace Loggers
{
    class LogMaster
    {
    private:
        typedef boost::variant<boost::shared_ptr<COutLogger>,boost::shared_ptr<CountingLogger>,boost::shared_ptr<UniformVectorLogger>,
                               boost::shared_ptr<ExternalProcessStatusLogger> > SupportedLoggers ;
        class ToStringVisitor
                : public boost::static_visitor<>
        {
        public:
            ToStringVisitor(const std::string& header) : m_header(header), m_result("") {}
            void operator()(boost::shared_ptr<COutLogger> logger)
            {
                m_result += logger->ToString(m_header);
            }
            void operator()(boost::shared_ptr<CountingLogger> logger)
            {
                m_result += logger->ToString(m_header);
            }
            void operator()(boost::shared_ptr<UniformVectorLogger> logger)
            {
                m_result += logger->ToString(m_header);
            }
            void operator()(boost::shared_ptr<ExternalProcessStatusLogger> logger)
            {
                m_result += logger->ToString(m_header);
            }
            std::string Result()
            {
                return m_result;
            }

        private:
            std::string m_header ;
            std::string m_result ;
        };

    public:
        typedef boost::shared_ptr<LogMaster> Ptr ;
        static Ptr Instance()
        {
            static Ptr _instance = Ptr(new LogMaster());

            return _instance ;
        }

        template<typename T>
        boost::shared_ptr<T> Get(const std::string& loggerName)
        {
            if(m_loggers.end() == m_loggers.find(loggerName))
            {
                m_loggers[loggerName] = SupportedLoggers(boost::shared_ptr<T>(new T(loggerName)));
            }
            return boost::get<boost::shared_ptr<T> >(m_loggers[loggerName]);
        }

        std::string ToString(const std::string& header)
        {
            std::string ret ;
            ToStringVisitor visitor(header) ;

            typedef std::map<std::string,SupportedLoggers>::const_iterator CI;

            for(CI p=m_loggers.begin();p!=m_loggers.end();++p)
            {
                boost::apply_visitor(visitor,p->second);
            }

            return visitor.Result();            
        }

    private:
        std::map<std::string,SupportedLoggers> m_loggers ;
    };
}

#endif // LOGMASTER_H
