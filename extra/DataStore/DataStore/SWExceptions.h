#ifndef SW_EXCEPTIONS_H
#define SW_EXCEPTIONS_H

///////////////////////////////////////////////////////////////////////////
// if you don't want, or cannot use exceptions, please uncomment one of the
// following line and make the other section empty
//#define DS_SW_ERROR(fmt, ...)
// or
//#define DS_SW_ERROR(fmt, ...) snprintf(sw_error_line, 200, "%s:%d:" fmt, __FILE__, __LINE__, ##__VA_ARGS__)
///////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////
#include <exception>
#include <string>

class DataStoreException : public std::exception
{
public:
    DataStoreException(const std::string& description) : m_description(description) {}
    ~DataStoreException() throw() {}
    const char* what() const throw()
    {
        return m_description.c_str() ;
    }
private:
    std::string m_description ;
};



#define DS_SW_ERROR(fmt, ...) \
    do \
    { \
        char sw_error_line[200];\
        snprintf(sw_error_line, 200, "%s:%d:" fmt, __FILE__, __LINE__, ##__VA_ARGS__); \
        throw DataStoreException(sw_error_line); \
    } while (false)

///////////////////////////////////////////////////////////////////////////

#endif // SW_EXCEPTIONS_H
