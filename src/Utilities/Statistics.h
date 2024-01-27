#include <memory>
#include <eigen3/Eigen/Dense>

template <typename T>
class Statistics
{
public:
    Statistics(size_t sample_size)
        : sample_size{sample_size}, index{0}
    {
        init();
    }

    bool ready()
    {
        return index == sample_size;
    }

    bool push_sample(T sample)
    {
        if(index == sample_size)
        {
            return true;
        }
        buffer[index++] = sample;

        if(index == sample_size)
        {
            return true;
        }
        return false;
    }

    T mean()
    {
        if(!ready())
        {
            return T();
        }
        T accumulator = buffer[0];
        for (size_t i = 1; i < sample_size; i++)
        {
            accumulator += buffer[i];
        }
        return accumulator / sample_size;
    }

    T variance()
    {
        if(!ready())
        {
            return T();
        }
        auto mean_value = mean();
        auto sum = sq(buffer[0]-mean_value);
        for (size_t i = 1; i < sample_size; i++)
        {
            sum += sq(buffer[i]-mean_value);
        }
        return sum / sample_size;
    }

    T sd()
    {
        if(!ready())
        {
            return T();
        }
        auto var = variance();
        return sqrt(var);
    }

    void reset()
    {
        index = 0;
        free();
        init();
    }

private:
    const size_t sample_size;
    size_t index;
    std::unique_ptr<T[]> buffer;

    void init()
    {
        buffer = std::move(std::make_unique<T[]>(sample_size));
    }

    void free()
    {
        if(buffer)
        {
            delete[] buffer.release();
        }
    }

    T sq(T val)
    {
        return val * val;
    }

    T sqrt(T val)
    {
        return std::sqrt(val);
    }
};

template <>
Eigen::Vector3f Statistics<Eigen::Vector3f>::sq(Eigen::Vector3f val)
{
    return val.cwisePow(2.0f);
}

template <>
Eigen::Vector3f Statistics<Eigen::Vector3f>::sqrt(Eigen::Vector3f val)
{
    return val.cwiseSqrt();
}


