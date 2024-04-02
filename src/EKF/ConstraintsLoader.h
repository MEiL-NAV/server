#pragma once
#include <dlfcn.h>
#include <eigen3/Eigen/Dense>
#include "../Utilities/Loggers/Logger.h"

class ConstraintsLoader 
{
public:   
    ConstraintsLoader()
        : lib(NULL), valid(false) 
    {
        Logger logger(LogType::ERROR);
        lib = dlopen("./libconstraints.so", RTLD_NOW);
        if (!lib) 
        {
            logger(std::string("Failed to open library: ") + dlerror());
            constraints_fun = NULL;
            constraints_derivative_fun = NULL;
        } 
        else 
        {
            constraints_fun = reinterpret_cast<constraints_handle>(
                dlsym(lib, "_Z11constraintsRKN5Eigen6MatrixIfLi15ELi1ELi0ELi15ELi1EEE"));
            
            constraints_derivative_fun = reinterpret_cast<constraints_derivative_handle>(
                dlsym(lib, "_Z22constraints_derivativeRKN5Eigen6MatrixIfLi15ELi1ELi0ELi15ELi1EEE"));    
        }

        if (!constraints_fun || !constraints_derivative_fun) 
        {
            logger("Failed to load functions: using default");
            constraints_fun = NULL;
            constraints_derivative_fun = NULL;
        }
        else
        {
            valid = true;
        }
    }

    Eigen::VectorXf
    constraints(const Eigen::Vector<float, 15> &state)
    {
        if(!constraints_fun)
            return Eigen::VectorXf::Zero(1);
        return constraints_fun(state);
    }

    Eigen::MatrixXf
    constraints_derivative(const Eigen::Vector<float, 15> &state)
    {
        if(!constraints_derivative_fun)
            return Eigen::MatrixXf::Ones(1,15);
        return constraints_derivative_fun(state);
    }

    bool is_valid() { return valid; }

    ~ConstraintsLoader() {
        if(lib)
        {
            dlclose(lib);
        }
    }



private:
    typedef Eigen::VectorXf
    (*constraints_handle) (const Eigen::Vector<float, 15> &state);

    typedef Eigen::MatrixXf
    (*constraints_derivative_handle) (const Eigen::Vector<float, 15> &state);

    constraints_handle constraints_fun;
    constraints_derivative_handle constraints_derivative_fun;
    void *lib;
    bool valid;

};