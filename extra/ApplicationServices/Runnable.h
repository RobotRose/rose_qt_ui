#ifndef RUNNABLE_H_
#define RUNNABLE_H_


// Runnable.h
// Copyright Sioux 2010
// Initial Creation date: Oct 12, 2010
//
// Description: An interface for every class that can be used with a ThreadRunner as its executor.

namespace ApplicationServices
{
    class Runnable
    {
    public:
        virtual ~Runnable() {}
        
        virtual bool prepare()=0;
        virtual bool runOnce()=0;
        virtual void finish()=0;

        virtual bool restart() { return false; };
    };
}


#endif /* RUNNABLE_H_ */
