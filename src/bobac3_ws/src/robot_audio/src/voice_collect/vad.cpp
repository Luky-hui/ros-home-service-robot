#include "vad.h"

VadAlgorithm::VadAlgorithm(size_t rate, uint8_t channels, uint8_t sample_length)
    :m_rate(rate), m_channels(channels), m_sample_length(sample_length) {}


size_t VadAlgorithm::get_nframes(uint8_t n_ms)
{
    return n_ms/1000.0 * m_rate * m_channels;
}


double VadAlgorithm::get_nframes_energy(short* nframes_buffer,size_t n_frames)
{
    double energy = 0.0;
    short sample;//sample_length = 16
    for (int i = 0; i<n_frames; i++) {
        sample = *(nframes_buffer + i);
        energy += sample * sample;
    }
    return log(energy);
}

short VadAlgorithm::sgn(int sample)
{
    return sample>=0?1:-1;
}


size_t VadAlgorithm::get_nframes_cross_zero_rate(short* nframes_buffer, size_t n_frames)
{
    int rate = 0;
    for (int i = 0; i<n_frames; i++) {
        if( sgn(*(nframes_buffer + i + 1)) - sgn( *(nframes_buffer + i)) ) {
            rate++;
        }
    }
    return rate;
}


bool VadAlgorithm::detect(boost::shared_ptr<char> buffer, size_t size, double EH, uint8_t n_ms)
{
    size_t nframes = get_nframes(n_ms);
    size_t  n_nframes = size / nframes;
    short* buf = (short*)buffer.get();
    m_EH = m_EL + EH;
    //printf("n_nframes:%d nframes:%d\n", n_nframes, nframes);
    int sum=0;

    for(size_t i=0; i<n_nframes; i++) {
        double energy = get_nframes_energy(buf + i*nframes, nframes);
        int rate = get_nframes_cross_zero_rate(buf + i*nframes, nframes);
        //printf("energy:%g EL:%g rate:%d ZL: %d\n", energy, m_EH, rate, m_ZL);
        if(energy > m_EH && rate > m_ZL)
            sum++;
    }
    //printf("sum: %d\n", sum);
    if(sum > 3 )
        return true;
    else
        return false;
}



void VadAlgorithm::env_detect(boost::shared_ptr<char> buffer, size_t size, uint8_t n_ms)
{
    size_t nframes = get_nframes(n_ms);
    size_t  n_nframes = size / nframes;
    printf("n_nframes:%ld nframes:%ld\n", n_nframes, nframes);
    short* buf = (short*)buffer.get();
    double E[n_nframes], E_sum=0.0, E_avg=0, E_std=0, Z_std=0;
    size_t Z[n_nframes], Z_sum=0, Z_avg=0;
    for(size_t i=0; i<n_nframes; i++) {
        E_sum += E[i] = get_nframes_energy(buf + i*nframes, nframes);
        Z_sum += Z[i] = get_nframes_cross_zero_rate(buf + i*nframes, nframes);
    }
    E_avg = E_sum / n_nframes;
    Z_avg = Z_avg / n_nframes;
    for(size_t i=0; i< n_nframes; i++) {
        E_std += (E[i] - E_avg) * (E[i] - E_avg);
        Z_std += (Z[i] - Z_avg) * (Z[i] - Z_avg);
    }
    m_E_std = E_std = sqrt(E_std);
    Z_std = sqrt(Z_std);
    m_EL = 18.02; //E_avg + m_E_std;
    m_ZL = Z_avg;
    std::cout << "EL: " << m_EL << std::endl;
    std::cout << "ZL: " << m_ZL << std::endl;
}


















