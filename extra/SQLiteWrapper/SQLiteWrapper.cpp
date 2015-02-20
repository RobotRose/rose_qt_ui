// SQLiteWrapper.cpp
// Copyright Sioux B.V. 2010
//
// SQLiteWrapper wraps the SQLite functionality visible to our system.

#include "SQLiteWrapper.h"
#include "SQLiteResult.h"
#include "EmptyResult.h"

#include <sqlite3.h>

#include <string>
#include <iostream>

using namespace std;
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <boost/shared_ptr.hpp>
using namespace boost;

SQLiteWrapper::SQLiteWrapper(const string& databaseFileName) throw(SQLiteOpenException)
        : m_needToFinalizeStatement(false)
{
    if(SQLITE_OK!=sqlite3_open_v2((databaseFileName).c_str(), &m_db, SQLITE_OPEN_READONLY, NULL))
    {
        throw SQLiteOpenException("Cannot open the database !") ;
    }
}

SQLiteWrapper::~SQLiteWrapper()
{
    FinalizeStatement();
    CloseDatabase();
}

void SQLiteWrapper::FinalizeStatement()
{
    if(m_needToFinalizeStatement)
    {
        if(SQLITE_OK!=sqlite3_finalize(m_statement))
        {
            std::cout << "[WARNING] SQLiteStatement not finalized properly !" << std::endl ;
        }
        m_needToFinalizeStatement = false;
    }
}

void SQLiteWrapper::CloseDatabase()
{
    if(SQLITE_OK!=sqlite3_close(m_db))
    {
        cout << "[WARNING] SQLite not released properly !" << endl ;
    }
}

void SQLiteWrapper::PepareStatement(const std::string & query) throw(ExecuteQueryException)
{
    FinalizeStatement();
    if(SQLITE_OK!=sqlite3_prepare_v2(
        m_db,                   /* Database handle */
        query.c_str(),          /* SQL statement, UTF-8 encoded */
        query.length(),         /* Maximum length of zSql in bytes. */
        &m_statement,             /* OUT: Statement handle */
        NULL                    /* OUT: Pointer to unused portion of zSql */
    ))
    {
        throw ExecuteQueryException("SQLliteWrapper::ExecuteQueryException()") ;
    }
    m_needToFinalizeStatement = true;
}

boost::shared_ptr<ApplicationServices::DatabaseResult> SQLiteWrapper::Step() throw(ExecuteQueryException)
{
    int sqlite3_result = sqlite3_step(m_statement);
    boost::shared_ptr<ApplicationServices::DatabaseResult> return_result ;
    if(SQLITE_ROW==sqlite3_result)
    {
        return_result = boost::shared_ptr<ApplicationServices::DatabaseResult>(new SQLiteResult(m_statement)) ;
    }
    else if(SQLITE_DONE==sqlite3_result)
    {
        return_result = boost::shared_ptr<ApplicationServices::DatabaseResult>(new EmptyResult()) ;
    }
    else
    {
        throw ExecuteQueryException("SQLliteWrapper::ExecuteQueryException()") ;
    }

    return return_result ;
}

boost::shared_ptr<ApplicationServices::DatabaseResult> SQLiteWrapper::GetNextResult() throw(ExecuteQueryException)
{
    return Step();
}

boost::shared_ptr<ApplicationServices::DatabaseResult> SQLiteWrapper::ExecuteQuery(const std::string& query) throw(ExecuteQueryException)
{
    PepareStatement(query);
    return GetNextResult() ;
}
