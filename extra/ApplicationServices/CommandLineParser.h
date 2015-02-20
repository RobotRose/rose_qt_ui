#ifndef COMMANDLINEPARSER_H
#define COMMANDLINEPARSER_H


#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <sstream>
#include <exception>

#include <boost/foreach.hpp>
#include <boost/variant.hpp>


namespace ApplicationServices
{
    class CommandLineParser
    {
    private:
        typedef boost::variant<int, std::string> AllowedArgumentTypes;
    public:
        class ParamNotFoundException : public std::exception
        {
        public:
            ParamNotFoundException(const std::string& paramName) : m_paramName(paramName) {}
            ~ParamNotFoundException() throw() {}
            const char* what() const throw()
            {
                return m_paramName.c_str() ;
            }
        private:
            std::string m_paramName ;
        };

        CommandLineParser(int argc, char **argv);

        CommandLineParser(int argc, char **argv, const std::string& rosSeparator);

        void List();

        bool IsSomeoneAskingForHelp();

        template <typename T>
        T GetParam(const std::string& name)
        {
            if(m_params.end() != m_params.find(name))
            {
                // we cannot make GetParam const because [] is not const operator !
                return boost::get<T>(m_params[name]);
            }
            else
            {
                throw ParamNotFoundException(name) ;
            }
        }

        int GetNonParsedArgc()
        {
            return m_rosParams.size();
        }

        const char ** GetNonParsedArgv()
        {
            return &m_rosParams[0] ;
        }

    private:
        std::map<std::string,AllowedArgumentTypes> m_params ;
        std::vector<const char*> m_rosParams ;
        std::string m_rosSeparator ;
        void AddParam(const std::string& param);
        bool ParamWithoutTheEqualitySign(const std::string& param);
        void ParseParam(const std::string& param);
        int ToInt(const std::string& paramValue);
        bool Number(const std::string& paramValue);
    };
};

#endif // COMMANDLINEPARSER_H
