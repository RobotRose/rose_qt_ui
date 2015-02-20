#ifndef UNIFORMVECTORLOGGER_H
#define UNIFORMVECTORLOGGER_H

#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>

#include <string>
#include <sstream>

namespace Loggers
{
    class UniformVectorLogger
    {
    public:
        typedef boost::shared_ptr<UniformVectorLogger> Ptr ;
        UniformVectorLogger(const std::string& name) : m_name(name), m_numberOfLogCalls(0), m_vectorValue(0), m_vectorSize(0) {}

        void Log(const std::vector<int8_t>& values)
        {
            if (!VectorSizeValid(values))
            {
                return ;
            }

            HandleFirstLogData(values) ;

            if(VectorSizeChanged(values) || VectorDataChanged(values))
            {
                return ;
            }

            AcceptLog();
        }

        int NumberOfLogCalls()
        {
            return m_numberOfLogCalls ;
        }

        int VectorValue()
        {
            return m_vectorValue ;
        }

        int VectorSize()
        {
            return m_vectorSize ;
        }

        std::string ToString(const std::string& header)
        {
            std::stringstream out ;

            out << header << "[" << m_name << "]" << " Received " << NumberOfLogCalls() << " vectors of size " << VectorSize() << " with value=" << VectorValue() << std::endl ;

            return out.str();
        }

    private:
        std::string m_name ;
        int m_numberOfLogCalls ;
        int m_vectorValue ;
        int m_vectorSize ;

        void HandleFirstLogData(const std::vector<int8_t>& values)
        {
            if(0 == m_vectorSize)
            {
             m_vectorSize = values.size();
            }

            if(0==m_vectorValue)
            {
             m_vectorValue = values[0] ;
            }
        }

        bool VectorSizeValid(const std::vector<int8_t>& values)
        {
            if (0==values.size())
            {
                return false;
            }
            else
            {
                return true ;
            }
        }

        bool VectorSizeChanged(const std::vector<int8_t>& values)
        {

            if((size_t)m_vectorSize != values.size())
            {
                return true ;
            }
            return false ;
        }

        bool VectorDataChanged(const std::vector<int8_t>& values)
        {
            BOOST_FOREACH(int value, values)
            {
                if(m_vectorValue != value)
                {
                    return true;
                }
            }
            return false ;
        }

        void AcceptLog()
        {
            m_numberOfLogCalls++;
        }
    };
};

#endif // UNIFORMVECTORLOGGER_H
