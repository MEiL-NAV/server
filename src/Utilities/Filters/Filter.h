#pragma once

template <typename T>
class Filter
{
public:

    virtual ~Filter() {}

    T get()
    {
        return last_value;
    }

    virtual T update(T sample, float delta_time) = 0;

    void reset(T val)
    {
        last_value = val;
    }

protected:
    T last_value;
};