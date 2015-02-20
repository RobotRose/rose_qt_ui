
// SQLiteResult.cpp
// Copyright Sioux 2010
// Initial Creation date: Sep 2, 2010
//
// Description: Holds Results returned from the SQLiteDatabase.

#include "SQLiteResult.h"

SQLiteResult::SQLiteResult(sqlite3_stmt* statement)
    : m_statement(statement)
{

}

Value SQLiteResult::GetValueForFieldAtIndex(int index) const
{
    switch(sqlite3_column_type(m_statement,index))
    {
        case SQLITE_INTEGER:
            return Value(sqlite3_column_int(m_statement, index));
        case SQLITE_TEXT:
            return Value(sqlite3_column_text(m_statement, index));
        case SQLITE_BLOB:
            return Value((const unsigned char*)sqlite3_column_blob(m_statement, index),sqlite3_column_bytes(m_statement, index));
        default:
            return Value(-1);
    }
}
