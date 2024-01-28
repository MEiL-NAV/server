#pragma once

#include <cmath>
#include <iostream>

template <typename T>
class LPF : public Filter<T>
{
public:

    LPF(float cutoff_freq)
        : cutoff_freq{cutoff_freq}
    {}

    virtual ~LPF() {}

    void set_cufoff_freq(float new_cutoff_freq)
    {
        cutoff_freq = new_cutoff_freq;
    }

    T update(T sample, float delta_time) override
    {
        auto p = exp(-cutoff_freq * delta_time);
        T new_value = last_value * p + sample * (1.0f - p);
        last_value = new_value;
        return new_value;
    }


private:
    T last_value;
    float cutoff_freq;
};