#ifndef SQLITERESULT_H_
#define SQLITERESULT_H_


// SQLiteResult.h
// Copyright Sioux 2010
// Initial Creation date: Sep 2, 2010
//
// Description: Header file for the SQLiteResult.cpp (see description there).

#include "ApplicationServices/DatabaseResult.h"

#include "sqlite3.h"

#include <boost/shared_ptr.hpp>

class SQLiteResult : public ApplicationServices::DatabaseResult
{
public:
    SQLiteResult(sqlite3_stmt* statement) ;
    ~SQLiteResult() {}
    Value GetValueForFieldAtIndex(int index) const;
    bool IsEmpty() const {return false;}
private:
    sqlite3_stmt* m_statement ;
};


#endif /* SQLITERESULT_H_ */
