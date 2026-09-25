#include "iostream"
#include "audio_capture.h"


AudioCap::AudioCap(std::string device_name)
{
    snd_pcm_sframes_t frames;
    if ( snd_pcm_open(&m_handle, device_name.c_str(), SND_PCM_STREAM_CAPTURE, 0) < 0) {
        printf("open mix error\n");
        exit(-1);
    }

    m_format = SND_PCM_FORMAT_S16;
    m_access_mode= SND_PCM_ACCESS_RW_INTERLEAVED;
    m_params=NULL;
    snd_pcm_hw_params_alloca(&m_params);
    snd_pcm_format_t m_format = SND_PCM_FORMAT_S16;    	/* sample format */
    snd_pcm_access_t m_access_mode= SND_PCM_ACCESS_RW_INTERLEAVED;
    m_rate = 16000;                       /* stream rate */
    m_channels = 1;                       /* count of channels */
    m_buffer_time = 500*1000;             /* ring buffer length in us */
    m_period_time = 100*1000;             /* period time in us */
    m_resample = 1;
    if(init()) {
        throw AudioException("failed to initalize audio device");
    }
    m_capture_flag = false;
    wait_flag = false;
}
AudioCap::~AudioCap()
{
    if(m_capture_flag) m_capture_flag = false;
    m_capture_thread.join();
    snd_pcm_close(m_handle);
}

void AudioCap::capture()
{
    size_t size = get_buffer_size();
    audio_buffer.clear();
    while(m_capture_flag) {
        boost::shared_ptr<char> tem_buffer (new char[size*2]);
        size_t frames = snd_pcm_readi(m_handle, tem_buffer.get(), size);
        boost::mutex::scoped_lock lock(m_mutex);
        audio_buffer.push_back(tem_buffer);
        if(wait_flag) {
            audio_buffer.erase(audio_buffer.begin());
        }
        if(audio_buffer.size() ==2 && !wait_flag) {
            m_cond_ready.notify_one();
        }
    }
}

boost::shared_ptr<char> AudioCap::get_current_frames()
{
    boost::shared_ptr<char> temp;
    boost::mutex::scoped_lock lock(m_mutex);
    m_cond_ready.wait(m_mutex);
    temp = audio_buffer.front();
    audio_buffer.erase(audio_buffer.begin());
    //std::cout << "audio_buffer.size(): " << audio_buffer.size() << std::endl;
    return temp;
}

bool AudioCap::start()
{
    m_capture_flag = true;
    m_capture_thread = boost::thread(boost::bind(&AudioCap::capture, this));
}
bool AudioCap::is_capture()
{
    return !m_capture_thread.timed_join(boost::get_system_time());
}

size_t AudioCap::get_buffer_size()
{
    return m_buffer_size;
}


int AudioCap::init()
{

    unsigned int rrate;
    snd_pcm_uframes_t size;
    int err, dir;
    /* choose all parameters */
    err = snd_pcm_hw_params_any(m_handle, m_params);
    if (err < 0) {
        printf("Broken configuration for playback: no configurations available: %s\n", snd_strerror(err));
        return err;
    }
    snd_pcm_hw_params_set_rate_resample(m_handle, m_params, m_resample);
    if (err < 0) {
        printf("Resampling setup failed for playback: %s\n", snd_strerror(err));
        return err;
    }
    /* set the interleaved read/write format */
    err = snd_pcm_hw_params_set_access(m_handle, m_params, m_access_mode);
    if (err < 0) {
        printf("Access type not available for playback: %s\n", snd_strerror(err));
        return err;
    }
    /* set the sample format */
    err = snd_pcm_hw_params_set_format(m_handle, m_params, m_format);
    if (err < 0) {
        printf("Sample format not available for playback: %s\n", snd_strerror(err));
        return err;
    }
    /* set the count of channels */
    err = snd_pcm_hw_params_set_channels(m_handle, m_params, m_channels);
    if (err < 0) {
        printf("Channels count (%i) not available for playbacks: %s\n", m_channels, snd_strerror(err));
        return err;
    }
    printf("set channels: %d\n", m_channels);
    /* set the stream rate */
    rrate = m_rate;
    err = snd_pcm_hw_params_set_rate_near(m_handle, m_params, &rrate, 0);
    if (err < 0) {
        printf("Rate %iHz not available for playback: %s\n", m_rate, snd_strerror(err));
        return err;
    }
    if (rrate != m_rate) {
        printf("Rate doesn't match (requested %iHz, get %iHz)\n", m_rate, err);
        return -EINVAL;
    }
    printf("set stream rate: %d\n", m_rate);
    /* set the buffer time */
    err = snd_pcm_hw_params_set_buffer_time_near(m_handle, m_params, &m_buffer_time, &dir);
    if (err < 0) {
        printf("Unable to set buffer time %i for playback: %s\n", m_buffer_time, snd_strerror(err));
        return err;
    }
    err = snd_pcm_hw_params_get_buffer_size(m_params, &size);
    if (err < 0) {
        printf("Unable to get buffer size for playback: %s\n", snd_strerror(err));
        return err;
    }
    /* set the period time */
    err = snd_pcm_hw_params_set_period_time_near(m_handle, m_params, &m_period_time, &dir);
    if (err < 0) {
        printf("Unable to set period time %i for playback: %s\n", m_period_time, snd_strerror(err));
        return err;
    }
    err = snd_pcm_hw_params_get_period_size(m_params, &size, &dir);
    if (err < 0) {
        printf("Unable to get period size for playback: %s\n", snd_strerror(err));
        return err;
    }

    m_period_size = size;
    m_buffer_size = 2*m_period_size;
    printf("period_size = %ld\n",m_period_size);
    printf("buffer_size = %ld\n",m_buffer_size);
    //buffer_size = size * 8;

    /* write the parameters to device */
    err = snd_pcm_hw_params(m_handle, m_params);
    if (err < 0) {
        printf("Unable to set hw params for playback: %s\n", snd_strerror(err));
        return err;
    }
    return 0;
}