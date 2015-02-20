/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*    Author: Mathijs de Langen
*    Date  : 2013/12/03
*         - File created.
*
* Description:
*    Thread runner for the Alsa services.
* 
***********************************************************************************/

#ifndef THREADRUNNER_H_
#define THREADRUNNER_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>

#include "ApplicationServices/Runnable.h"
#include <string>

namespace ApplicationServices
{
    class ThreadRunner
    {
    public:
        typedef boost::shared_ptr<ThreadRunner> Ptr ;
        ThreadRunner(boost::shared_ptr<ApplicationServices::Runnable> executor, const std::string& name ) ;
        ~ThreadRunner() ;
        template<typename TimeDuration>
        bool IsFinished(const TimeDuration& rel_time) { return m_boostThread.timed_join(rel_time); }

    private:
        void Run();
        bool m_stopped ;
        std::string m_Name;
        boost::shared_ptr<ApplicationServices::Runnable> m_executor ;
        boost::thread m_boostThread ;
    };
};

#endif /* THREADRUNNER_H_ */
