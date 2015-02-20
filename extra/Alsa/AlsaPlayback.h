/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*    Author: Mathijs de Langen
*    Date  : 2013/12/03
*         - File created.
*
* Description:
*    Audio player.
* 
***********************************************************************************/

#ifndef ALSAPLAYBACK_H_
#define ALSAPLAYBACK_H_

#include <deque>

/* Use the newer ALSA API */
#define ALSA_PCM_NEW_HW_PARAMS_API

#include <alsa/asoundlib.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include "std_msgs/ByteMultiArray.h"

#include "ApplicationServices/Runnable.h"
#include "ApplicationServices/ListenerImplementation.h"

namespace Alsa
{
    class AlsaPlayback : public ApplicationServices::ListenerImplementation<const std_msgs::ByteMultiArray::ConstPtr>, public ApplicationServices::Runnable
    {
    private:
        typedef ApplicationServices::Listener<const std_msgs::ByteMultiArray::ConstPtr>::Ptr ListenerConstPtr ;
        static const snd_pcm_uframes_t FRAMES = 2048;
        static const int DEFAULTBUFFERINGDEPTH = 3;
    public:
        typedef boost::shared_ptr<AlsaPlayback> Ptr ;

        AlsaPlayback(ListenerConstPtr Listener);

        // The Runnable interface
        bool prepare();
        bool runOnce();
        void finish();

        // Alsa Playback public interface
        void setBufferingDepth(int bufferingDepth)
        {
            m_bufferingDepth = bufferingDepth ;
        }
    private:
        void enqueue(const std_msgs::ByteMultiArray::ConstPtr& frame);
        void setupAlsa();

        // The ApplicationServices::ListenerImplementation interface
        void CallbackImplementation(const std_msgs::ByteMultiArray::ConstPtr& param);

        ListenerConstPtr m_Listener ;
        boost::mutex m_mutex ;
        std::deque<std_msgs::ByteMultiArray::ConstPtr> m_queue ;
        snd_pcm_t* m_handle;
        int m_bufferingDepth ;
    } ;
};


#endif /* ALSAPLAYBACK_H_ */
