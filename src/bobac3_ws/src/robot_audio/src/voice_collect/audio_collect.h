#ifndef AUDIO_COLLECT_H
#define AUDIO_COLLECT_H

#include "audio_capture.h"
#include "vad.h"
#include "boost/thread/thread.hpp"
#include "boost/bind.hpp"
#include "boost/thread/mutex.hpp"
#include "boost/thread/condition.hpp"

using namespace std;
class AudioCollect
{
public:
    AudioCollect();
    virtual ~AudioCollect()
    {
        if(m_work_flag) m_work_flag = false;
        m_work_thread.join();
        if(m_vad) delete m_vad;
        if(m_cap) delete m_cap;
    }
    AudioCap* m_cap;
    VadAlgorithm* m_vad;
    std::vector<boost::shared_ptr<char> > m_collecter;
    bool m_collect_flag;
    std::vector<boost::shared_ptr<char> > get_voice();
    void write_wav(std::vector<boost::shared_ptr<char> > voice, std::string file_name);

private:

    boost::thread m_work_thread;
    bool m_work_flag;
    void work();
    std::vector<boost::shared_ptr<char> > m_voice;
    boost::mutex m_mutex;
    boost::condition m_cond;
    //boost::thread m_collect_thread;
    //void collect_thread();


};

#endif //AUDIO_COLLECT_H