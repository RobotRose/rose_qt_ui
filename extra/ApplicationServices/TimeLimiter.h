#ifndef TIMELIMITER_H
#define TIMELIMITER_H

#include "ApplicationServices/Runnable.h"

namespace ApplicationServices
{
    class TimeLimiter : public Runnable
    {
    private:
        int m_timeToRunInSec ;
        bool m_finished ;
    public:
        typedef boost::shared_ptr<TimeLimiter> Ptr ;
        TimeLimiter(int timeToRunInSec=-1)
            : m_timeToRunInSec(timeToRunInSec), m_finished(false)
        {
        }

        bool finished()
        {
            return m_finished ;
        }

        virtual bool prepare()
        {
            if(-1 != m_timeToRunInSec)
            {
                std::cout << "TimeLeft:" << m_timeToRunInSec << std::endl;
            }

            sleep(1);

            if(m_timeToRunInSec > 0)
            {
                if(0 == --m_timeToRunInSec)
                {
                    m_finished = true ;
                    return true;
                }
            }
            return false ;
        }

        virtual bool runOnce()
        {
            return false;
        }

        virtual void finish()
        {
        }

    };
};

#endif // TIMELIMITER_H
