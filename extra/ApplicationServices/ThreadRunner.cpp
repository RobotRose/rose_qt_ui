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
// Description: A convenience thread runner.

#include "ApplicationServices/ThreadRunner.h"
#include <iostream>

ApplicationServices::ThreadRunner::ThreadRunner(boost::shared_ptr<ApplicationServices::Runnable> executor, const std::string& name )
    : m_stopped( false )
    , m_executor( executor )
	, m_boostThread( &ThreadRunner::Run, this)
    , m_Name( name )
{
}

ApplicationServices::ThreadRunner::~ThreadRunner()
{
    m_stopped = true ;
    m_boostThread.timed_join(boost::posix_time::seconds(1));
}

void ApplicationServices::ThreadRunner::Run()
{
    while(!m_stopped && !m_executor->prepare())
    {
    }

    while(!m_stopped)
    {
        if(m_executor->restart())
        {
            m_executor->finish();

            while(!m_stopped && !m_executor->prepare())
            {   
            }
        }
        
        if(m_executor->runOnce())
        {
            break;
        }
    }

    m_executor->finish();
}
