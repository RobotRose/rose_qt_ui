#ifndef ALSACAPTURE_H_
#define ALSACAPTURE_H_


// AlsaPublisher.h
// Copyright Sioux 2010
// Initial Creation date: Oct 12, 2010
//
// Description: Alsa Capture device wrapper.

/* Use the newer ALSA API */
#define ALSA_PCM_NEW_HW_PARAMS_API

#include <alsa/asoundlib.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include "std_msgs/ByteMultiArray.h"

#include "ApplicationServices/Runnable.h"
#include "Ros/RosWrapper.h"

namespace Alsa
{
    class AlsaCapture : public ApplicationServices::Runnable
    {
    public:
        typedef boost::shared_ptr<AlsaCapture> Ptr ;

        AlsaCapture(std::string topicname, Ros::RosWrapper::Ptr rosWrapper);

        bool prepare();
        bool runOnce();
        void finish();
    private:
        std::string m_OutTopic;
        static const snd_pcm_uframes_t FRAMES = 2048;
        Ros::RosWrapper::Ptr m_rosWrapper;
        int m_size;
        snd_pcm_t* m_handle;
        snd_pcm_uframes_t m_frames;
        std_msgs::ByteMultiArray m_frame ;
        void setupAlsa();
    } ;
};


#endif /* ALSACAPTURE_H_ */
