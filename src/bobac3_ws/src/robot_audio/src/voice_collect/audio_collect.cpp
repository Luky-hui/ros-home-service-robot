#include "audio_collect.h"

#include "wav.h"

AudioCollect::AudioCollect():m_cap(new AudioCap("default")), m_vad(new VadAlgorithm(16000, 1, 16))
{
    if(m_cap) {
        m_cap->start();
        usleep(30*1000);
    }
    if(m_vad && m_cap->is_capture()) {
        m_vad->env_detect(m_cap->get_current_frames(), m_cap->get_buffer_size(), 20);
    }
    m_cap->wait_flag = true;
    m_work_flag = true;
    m_work_thread = boost::thread(boost::bind(&AudioCollect::work, this));

}

void AudioCollect::work()
{
    m_collect_flag=false;
    int no_sound_sum=0;
    int sound_num=0;
    size_t size = m_cap->get_buffer_size();
    std::vector<uint8_t> res_vec;
    while(m_work_flag) {
        boost::mutex::scoped_lock lock(m_mutex);
        boost::shared_ptr<char> tem = m_cap->get_current_frames();
        m_collecter.push_back(tem);
        if(!m_collect_flag && m_collecter.size()> 2) {
            m_collecter.erase(m_collecter.begin());
        }
        bool res = m_vad->detect(tem, size, 0, 20);
        if(res)
            res_vec.push_back(1);
        else
            res_vec.push_back(0);
        std::vector<uint8_t>::reverse_iterator it = res_vec.rbegin();
        if(res_vec.size()>4) {
            if((*it)>0 && *(it+1)>0 /*&& *(it+2)>0*/)	{
                m_collect_flag = true;
            }

            if((*it)==0 && *(it+1)==0 && *(it+2)==0 /*&& (it+3)==0 && *(it+4)==0*/) {
                m_collect_flag = false;
            }
            res_vec.erase(res_vec.begin());
            //std::cout << "res_vec.size(): " << res_vec.size() << std::endl;
        }

        if(m_collect_flag) {
            printf("record------------\n");
        } else {
            //printf("over/no sound=====\n");
        }

    }
}

std::vector<boost::shared_ptr<char> > AudioCollect::get_voice()
{
    size_t  size = m_cap->get_buffer_size();
    bool rec_flag = false;
    m_voice.clear();
    m_cap->wait_flag = false;
    while(true) {
        if(m_collect_flag) {
            rec_flag = true;
        }
        if(rec_flag) {
            printf("****************************\n");
            if(!m_collect_flag) {
                std::cout << "collecter size: " << m_collecter.size() << std::endl;
                m_voice = m_collecter;
                break;
            }
        }
        usleep(1000);
    }
    m_cap->wait_flag = true;
    m_collecter.clear();
    return m_voice;
}

void AudioCollect::write_wav(std::vector<boost::shared_ptr<char> > voice, std::string fname)
{
    std::string	file_name = fname;
    size_t size = m_cap->get_buffer_size();
    FILE* fp = fopen(file_name.c_str(), "wb+");
    fwrite(&default_wav_hdr, sizeof(default_wav_hdr) ,1, fp);
    for(int i=0; i<voice.size(); i++) {
        char* buf = voice[i].get();
        if(buf) {
            fwrite(buf, 2*size, 1, fp);
            default_wav_hdr.data_size += 2*size;
        }
    }

    default_wav_hdr.size_8 += default_wav_hdr.data_size + (sizeof(default_wav_hdr) - 8);
    fseek(fp, 4, 0);
    fwrite(&default_wav_hdr.size_8,sizeof(default_wav_hdr.size_8), 1, fp); //写入size_8的值
    fseek(fp, 40, 0); //将文件指针偏移到存储data_size值的位置
    fwrite(&default_wav_hdr.data_size,sizeof(default_wav_hdr.data_size), 1, fp); //写入data_size的值

    fclose(fp);
}

