
// AlsaPublisher.cpp
// Copyright Sioux 2010
// Initial Creation date: Oct 12, 2010
//
// Description: Alsa publisher. Performs the audio capturing from the default input device.

#include "Alsa/AlsaCapture.h"
#include "std_msgs/ByteMultiArray.h"

Alsa::AlsaCapture::AlsaCapture(std::string topicname, Ros::RosWrapper::Ptr rosWrapper)
    : m_rosWrapper(rosWrapper)
	, m_OutTopic( topicname )
{
    setupAlsa();
}

bool Alsa::AlsaCapture::prepare()
{
    m_frame.data.resize(m_size);
    return true;
}

bool Alsa::AlsaCapture::runOnce()
{
    snd_pcm_readi(m_handle, &(m_frame.data)[0], m_frames);

    m_rosWrapper->PublishToTopic( m_OutTopic, m_frame);

    return false ;
}

void Alsa::AlsaCapture::finish()
{
    snd_pcm_close(m_handle);
}

void Alsa::AlsaCapture::setupAlsa()
{
    int rc;
    snd_pcm_hw_params_t *params;
    unsigned int val;
    int dir;

    /* Open PCM device for recording (capture). */
    rc = snd_pcm_open(&m_handle, "default",
                    SND_PCM_STREAM_CAPTURE, 0);

    /* Allocate a hardware parameters object. */
    snd_pcm_hw_params_alloca(&params);

    /* Fill it in with default values. */
    snd_pcm_hw_params_any(m_handle, params);

    /* Set the desired hardware parameters. */

    /* Interleaved mode */
    snd_pcm_hw_params_set_access(m_handle, params,  SND_PCM_ACCESS_RW_INTERLEAVED);
    //snd_pcm_hw_params_set_access(m_handle, params,  SND_PCM_ACCESS_RW_NONINTERLEAVED);


    /* Signed 16-bit little-endian format */
    snd_pcm_hw_params_set_format(m_handle, params, SND_PCM_FORMAT_S16_LE);
    //snd_pcm_hw_params_set_format(m_handle, params, SND_PCM_FORMAT_S8 );

    /* Two channels (stereo) */
    snd_pcm_hw_params_set_channels(m_handle, params, 1);

    /* 44100 bits/second sampling rate (CD quality) */
    val = 8000;
    snd_pcm_hw_params_set_rate_near(m_handle, params,
                                  &val, &dir);

    /* Set period size to 32 frames. */
    m_frames = FRAMES;
    snd_pcm_hw_params_set_period_size_near(m_handle,
                              params, &m_frames, &dir);

    /* Write the parameters to the driver */
    rc = snd_pcm_hw_params(m_handle, params);

    /* Use a buffer large enough to hold one period */
    snd_pcm_hw_params_get_period_size(params,
                                      &m_frames, &dir);
    m_size = m_frames * 4; /* 2 bytes/sample, 2 channels */
}
