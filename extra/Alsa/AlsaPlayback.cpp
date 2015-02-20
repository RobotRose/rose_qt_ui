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
#include "Alsa/AlsaPlayback.h"

Alsa::AlsaPlayback::AlsaPlayback(ListenerConstPtr Listener)
    : m_Listener(Listener), m_bufferingDepth(DEFAULTBUFFERINGDEPTH)
{
    setupAlsa();
}

void Alsa::AlsaPlayback::CallbackImplementation(const std_msgs::ByteMultiArray::ConstPtr& param)
{
	enqueue(param);
}

bool Alsa::AlsaPlayback::prepare()
{
    usleep(100);
    return (m_bufferingDepth < m_queue.size());
}

bool Alsa::AlsaPlayback::runOnce()
{
    m_mutex.lock();

    if(!m_queue.empty())
    {
        snd_pcm_writei(m_handle, &(m_queue.front()->data[0]), FRAMES);
        m_queue.pop_front();
    }

    m_mutex.unlock();
}

void Alsa::AlsaPlayback::finish()
{
    snd_pcm_close(m_handle);
}


void Alsa::AlsaPlayback::enqueue(const std_msgs::ByteMultiArray::ConstPtr& frame)
{
    m_mutex.lock();
    m_queue.push_back(frame);
    m_mutex.unlock();
}

void Alsa::AlsaPlayback::setupAlsa()
{
    int rc;
    int size;
    snd_pcm_hw_params_t *params;
    unsigned int val;
    int dir;
    snd_pcm_uframes_t frames;

    /* Open PCM device for playback. */
    rc = snd_pcm_open(&m_handle, "default",
                    SND_PCM_STREAM_PLAYBACK, 0);

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
    frames = FRAMES;
    snd_pcm_hw_params_set_period_size_near(m_handle,
                              params, &frames, &dir);

    /* Write the parameters to the driver */
    rc = snd_pcm_hw_params(m_handle, params);
}
