#pragma once
#include <numbers>

class Converters
{
public:

    template <typename T>
    static T mdeg_to_radians(T val_in_mdeg)
    {
        return val_in_mdeg * 0.001f * std::numbers::pi / 180.0f;
    }
};