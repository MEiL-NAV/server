#pragma once
#include <dlfcn.h>
#include <eigen3/Eigen/Dense>
#include <iostream>

class ConstraintsLoader 
{
public:   
    ConstraintsLoader() {
        lib = dlopen("./libconstraints.so", RTLD_NOW);
        if (!lib) 
        {
            std::cerr << "Failed to open library: " << dlerror() << std::endl;
            constraints_fun = NULL;
            constraints_derivative_fun = NULL;
        } 
        else 
        {
            constraints_fun = reinterpret_cast<constraints_handle>(
                dlsym(lib, "_Z11constraintsRKN5Eigen6MatrixIfLi13ELi1ELi0ELi13ELi1EEE"));
            
            constraints_derivative_fun = reinterpret_cast<constraints_derivative_handle>(
                dlsym(lib, "_Z22constraints_derivativeRKN5Eigen6MatrixIfLi13ELi1ELi0ELi13ELi1EEE"));    
        }
        if (!constraints_fun) 
        {
            std::cerr << "Failed to load function constraints: using default"  << std::endl;
        }
        if (!constraints_derivative_fun) 
        {
            std::cerr << "Failed to load function constraints_derivative: using default" << std::endl;
        }
    }

    Eigen::Vector<float, 13>
    constraints(const Eigen::Vector<float, 13> &state)
    {
        if(!constraints_fun)
            return Eigen::Vector<float, 13>::Zero();
        return constraints_fun(state);
    }

    Eigen::Matrix<float, 13, 13>
    constraints_derivative(const Eigen::Vector<float, 13> &state)
    {
        if(!constraints_derivative_fun)
            return Eigen::Matrix<float, 13, 13>::Identity();
        return constraints_derivative_fun(state);
    }

    ~ConstraintsLoader() {
        if(lib)
        {
            dlclose(lib);
        }
    }



private:
    typedef Eigen::Vector<float, 13>
    (*constraints_handle) (const Eigen::Vector<float, 13> &state);

    typedef Eigen::Matrix<float, 13, 13>
    (*constraints_derivative_handle) (const Eigen::Vector<float, 13> &state);

    constraints_handle constraints_fun;
    constraints_derivative_handle constraints_derivative_fun;
    void *lib;

};