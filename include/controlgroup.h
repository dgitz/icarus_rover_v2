#ifndef CONTROLGROUP_H
#define CONTROLGROUP_H

#include "ros/ros.h"
#include "Definitions.h"
#include "ros/time.h"
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <eros/diagnostic.h>
#include <eros/signal.h>
#include <eros/pin.h>

/*
This Class represents a library that takes in a series of inputs, and calculates 1 output based on the input data and various config items.


 */
class ControlGroup
{
public:
    enum class Mode
	{
		UNKNOWN=0,
		PID=1, //2 Input Signals of Type: SIGNAL are Required.  No other Input Signals are available.
	};
    enum class SignalClass
    {
        UNKNOWN=0,
        SIGNAL=1,
        PIN=2
    };
    enum class SignalDirection
    {
        UNKNOWN=0,
        INPUT=1,
        OUTPUT=2
    };
    struct Input
    {
        bool properties_initialized;
        std::string name;
        SignalClass signal_class;
        eros::signal signal;
        eros::pin pin;
        uint64_t rx_counter;
    };
    struct Output
    {
        bool properties_initialized;
        std::string name;
        SignalClass signal_class;
        eros::signal signal;
        eros::pin pin;
    };
	ControlGroup();
    ~ControlGroup();
    
    /*  Init Functions */
    eros::diagnostic init(eros::diagnostic diag,Mode t_mode,std::string _name);
    bool set_PIDGains(double t_P,double t_I,double t_D);
    // Checks all initialize values and determines if this is a valid configuration or not.
    eros::diagnostic finish_initialization();

    // Initialize Signal and insert into Input/Output as appropriate.
    eros::diagnostic initialize_signal(SignalDirection dir,std::string name,uint8_t type);
     // Initialize Pin and insert into Input/Output as appropriate.
    eros::diagnostic initialize_pin(SignalDirection dir,
        std::string connected_device);
    eros::diagnostic set_pinproperties(eros::pin pin);

    /* Attribute Functions */
    std::string get_name() { return name; }
    eros::diagnostic get_diagnostic() { return diagnostic; }
    bool is_initialized() { return initialized; }
    bool is_ready() { return ready; }
    std::vector<std::string> get_inputsignalnames()
    {
        std::vector<std::string> names;
        for(std::size_t i = 0; i < inputs.size(); ++i)
        {
            names.push_back(inputs.at(i).signal.name);
        }
        return names;
    }
    eros::signal get_inputsignal(std::string name)
    {
        for(std::size_t i = 0; i < inputs.size(); ++i)
        {
            if(inputs.at(i).signal.name == name)
            {
                return inputs.at(i).signal;
            }
        }
        eros::signal empty_signal;
        return empty_signal;
    }
    std::vector<eros::pin> get_outputpins()
    {
        std::vector<eros::pin> pins;
        for(std::size_t i = 0; i < outputs.size(); ++i)
        {
            pins.push_back(outputs.at(i).pin);
        }
        return pins;
    }
    std::vector<eros::signal> get_outputsignals()
    {
        std::vector<eros::signal> signals;
        for(std::size_t i = 0; i < outputs.size(); ++i)
        {
            signals.push_back(outputs.at(i).signal);
        }
        return signals;
    }
    void set_outputlimits(double min,double max)
    {
        output_max = max;
        output_min = min;
    }
    void set_max_deltaoutput(double d_output) //(Current Output-Last Output) / Sampling Time
    {
        delta_output_limit = d_output;
    }
    /* Update Functions */
    eros::diagnostic new_input(eros::signal signal);
    eros::diagnostic update(double dt);
    void reset_integral()
    {
        integral_error = 0;
        integral_reset_counter++;
    }
private:
    std::string name;
    eros::diagnostic diagnostic;
    std::vector<Input> inputs;
    std::vector<Output> outputs;
    Mode mode;
    bool initialized;
    bool ready;
    double run_time;
    double gain_P;
    double gain_I;
    double gain_D;
    uint64_t integral_reset_counter;
    double error;
    double integral_error;
    double output_max;
    double output_min;
    double delta_output_limit;
   
};
#endif
