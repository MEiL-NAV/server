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


private:
    T last_value;
};