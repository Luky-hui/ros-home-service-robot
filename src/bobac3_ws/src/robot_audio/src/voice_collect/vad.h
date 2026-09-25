#ifndef VAD_H
#define VAD_H
//voice activity detection
#include "stdio.h"
#include "iostream"
#include "boost/thread/thread.hpp"
#include "boost/bind.hpp"
#include "boost/thread/mutex.hpp"
#include "boost/thread/condition.hpp"
#include "cmath"

class VadAlgorithm
{
public:
    VadAlgorithm(size_t rate=16000, uint8_t channels=1, uint8_t sample_length=16);
    virtual ~VadAlgorithm() {}
    bool detect(boost::shared_ptr<char> buffer, size_t size, double EH, uint8_t n_ms=20);

    void env_detect(boost::shared_ptr<char> buffer, size_t size, uint8_t n_ms=20);


    void start();
private:
    size_t m_rate;
    uint8_t m_channels;
    uint8_t m_sample_length;

    size_t get_nframes(uint8_t n_ms=20);
    double get_nframes_energy(short* nframes_buffer, size_t n_frames);

    short sgn(int sample);

    size_t get_nframes_cross_zero_rate(short* nframes_buffer, size_t n_frames);

    double m_EL, m_EH, m_E_std;
    size_t m_ZL, m_ZH;
};


#endif //VAD_H
