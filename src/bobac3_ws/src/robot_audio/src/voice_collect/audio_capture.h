#ifndef AUDIO_CAPTURE_H
#define AUDIO_CAPTURE_H
#include "string"


#include <alsa/asoundlib.h>
#include <vector>
#include "boost/thread/thread.hpp"
#include "boost/bind.hpp"
#include "boost/thread/mutex.hpp"
#include "boost/thread/condition.hpp"
/*
#include "wav.h"
*/
using namespace std;
class AudioException:public std::exception
{
public:
    explicit AudioException(std::string error)throw():m_error(error) {}
    const char* what()throw()
    {
        return m_error.c_str();
    }
    virtual ~AudioException()throw() {}
private:
    std::string m_error;
};

class AudioCap
{
public:

    AudioCap(std::string device_name="default");
    ~AudioCap();
    bool start();
    size_t get_buffer_size();
    bool is_capture();
    boost::shared_ptr<char> get_current_frames();
    void record_start();
    void record_stop();
    bool wait_flag;
private:
    int init();
    void capture();
    void record();
    snd_pcm_format_t m_format;    					/* sample format */
    snd_pcm_access_t m_access_mode;
    unsigned int m_rate;                    /* stream rate */
    unsigned int m_channels;                /* count of channels */
    unsigned int m_buffer_time;             /* ring buffer length in us */
    unsigned int m_period_time;             /* period time in us */
    unsigned int m_resample;

    snd_pcm_sframes_t m_buffer_size;
    snd_pcm_sframes_t m_period_size;
    snd_pcm_t* m_handle;
    snd_pcm_hw_params_t* m_params;

    std::vector<boost::shared_ptr<char> > audio_buffer;
    boost::thread m_record_thread;
    bool m_record_flag;

    boost::shared_ptr<char> m_current_frames;

    boost::thread m_capture_thread;
    bool m_capture_flag;
    boost::mutex m_mutex;
    boost::condition m_cond_empty;
    boost::condition m_cond_ready;
    FILE* m_fp;


};


#endif //AUDIO_CAPTURE_H