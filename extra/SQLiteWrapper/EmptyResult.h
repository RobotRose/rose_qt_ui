#ifndef EMPTYRESULT_H_
#define EMPTYRESULT_H_


// EmptyResult.h
// Copyright Sioux 2010
// Initial Creation date: Sep 6, 2010
//
// Description: Empty result is a result that does not contain anything.

#include "ApplicationServices/DatabaseResult.h"
#include "Value.h"

class EmptyResult : public ApplicationServices::DatabaseResult
{
public:
    ~EmptyResult() {}
    Value GetValueForFieldAtIndex(int index) const
    {
        return Value();
    }
    bool IsEmpty() const {return true;}
};

#endif /* EMPTYRESULT_H_ */
