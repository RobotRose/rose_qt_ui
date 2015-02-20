#include "ApplicationServices/CommandLineParser.h"

ApplicationServices::CommandLineParser::CommandLineParser(int argc, char **argv)
{
    if(1 < argc)
    {
        for(int i=1; i<argc; i++)
        {
            AddParam(argv[i]);
        }
    }
}

ApplicationServices::CommandLineParser::CommandLineParser(int argc, char **argv, const std::string& rosSeparator)
    : m_rosSeparator(rosSeparator)
{
    if(1 < argc)
    {
        for(int i=1; i<argc; i++)
        {
            if(m_rosSeparator == argv[i])
            {
                if(argc >= i+1)
                {
                    m_rosParams = std::vector<const char*>(&argv[i+1],&argv[argc]);
                    m_rosParams.insert(m_rosParams.begin(),argv[0]);
                }
                break;
            }
            AddParam(argv[i]);
        }
    }
}

void ApplicationServices::CommandLineParser::List()
{
    typedef std::pair<std::string,AllowedArgumentTypes> KeyValue ;
    BOOST_FOREACH(KeyValue keyValue, m_params)
    {
        std::cout << "params[" << keyValue.first << "]=" << keyValue.second << std::endl;
    }
}

bool ApplicationServices::CommandLineParser::IsSomeoneAskingForHelp()
{
    if(1 == boost::get<int>(m_params["help"]))
        return true ;
    else
        return false ;
}

bool ApplicationServices::CommandLineParser::ParamWithoutTheEqualitySign(const std::string& param)
{
    return (std::string::npos == param.find_first_of("="));
}

bool ApplicationServices::CommandLineParser::Number(const std::string& paramValue)
{
    std::stringstream ss(paramValue);

    int intValue ;
    if(!((ss >> intValue).fail()))
    {
        if(ss.eof())
        {
            return true ;
        }
    }
    return false ;
}

int ApplicationServices::CommandLineParser::ToInt(const std::string& paramValue)
{
    std::stringstream ss(paramValue);

    int intValue ;
    ss >> intValue ;

    return intValue ;
}

void ApplicationServices::CommandLineParser::ParseParam(const std::string& param)
{
    size_t equalitySignPos = param.find_first_of("=");

    std::string paramName = param.substr(0,equalitySignPos);
    std::string paramValue = param.substr(equalitySignPos+1);

    if(Number(paramValue))
    {
        m_params[paramName] = AllowedArgumentTypes(ToInt(paramValue)) ;
    }
    else
    {
        m_params[paramName] = AllowedArgumentTypes(paramValue) ;
    }
}

void ApplicationServices::CommandLineParser::AddParam(const std::string& param)
{
    if(ParamWithoutTheEqualitySign(param))
    {
        m_params[param] = AllowedArgumentTypes(1);
    }
    else
    {
        ParseParam(param);
    }
}
