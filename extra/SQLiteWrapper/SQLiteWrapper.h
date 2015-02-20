#ifndef SQLITEWRAPPER_H_
#define SQLITEWRAPPER_H_

// SQLiteWrapper.h
// Copyright Sioux B.V. 2010
//
// SQLiteWrapper wraps the SQLite functionality visible to our system.

#include "sqlite3.h"

#include "ApplicationServices/Database.h"

#include <string>
#include <boost/shared_ptr.hpp>

class SQLiteWrapper : public ApplicationServices::Database
{
public:
    typedef boost::shared_ptr<SQLiteWrapper> Ptr ;
    class SQLiteOpenException : public std::exception
    {
    public:
        SQLiteOpenException(const std::string& message)
        {
            m_message = message ;
        }
        ~SQLiteOpenException() throw()
        {
        }
        const char* what() const throw()
        {
            return m_message.c_str();
        }
    private:
        std::string m_message ;
    };

    class ExecuteQueryException : public std::exception
    {
    public:
        ExecuteQueryException(const std::string& message)
        {
            m_message = message ;
        }
        ~ExecuteQueryException() throw()
        {
        }
        const char* what() const throw()
        {
            return m_message.c_str();
        }
    private:
        std::string m_message ;
    };

    explicit SQLiteWrapper(const std::string& databaseFileName) throw(SQLiteOpenException);
    ~SQLiteWrapper();
    boost::shared_ptr<ApplicationServices::DatabaseResult> ExecuteQuery(const std::string& query) throw(ExecuteQueryException) ;
    boost::shared_ptr<ApplicationServices::DatabaseResult> GetNextResult() throw(ExecuteQueryException) ;
private:
    std::string home_dir_;
    
    sqlite3 *m_db;
    sqlite3_stmt *m_statement ;
    bool m_needToFinalizeStatement ;
    boost::shared_ptr<ApplicationServices::DatabaseResult> Step() throw(ExecuteQueryException) ;
    void CloseDatabase();
    void FinalizeStatement();
    void PepareStatement(const std::string & query) throw(ExecuteQueryException) ;
};


#endif /* SQLITEWRAPPER_H_ */
