#pragma once

#include <cmath>
#include <iostream>
#include "Filter.h"

template <typename T>
class LPF : public Filter<T>
{
public:

    LPF(float cutoff_freq)
        :   Filter<T>(),
            cutoff_freq{cutoff_freq}
    {}

    virtual ~LPF() {}

    void set_cufoff_freq(float new_cutoff_freq)
    {
        cutoff_freq = new_cutoff_freq;
    }

    T update(T sample, float delta_time) override
    {
        auto p = exp(-cutoff_freq * delta_time);
        T new_value = this->last_value * p + sample * (1.0f - p);
        this->last_value = new_value;
        return new_value;
    }

private:
    float cutoff_freq;
};