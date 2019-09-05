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
#include <eros/controlgroup.h>
#include <eros/signal.h>

class ControlGroup
{
public:
    enum class Mode
	{
		UNKNOWN=0,
		PID=1,
	};
	ControlGroup();
    ~ControlGroup();

    /*  Init Functions */
    eros::diagnostic init(eros::diagnostic diag,Mode t_mode);
    bool set_PIDGains(double t_P,double t_I,double t_D);
    /* Attribute Functions */
    eros::controlgroup get_controlgroupmsg() { return cg; }
    eros::diagnostic get_diagnostic() { return diagnostic; }
    bool is_initialized() { return initialized; }
    bool is_ready() { return ready; }
    eros::signal get_output() { return output; }
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
    eros::diagnostic set_inputvalue(double v)
    {
        eros::diagnostic diag = diagnostic;
        inputvalue_received_count++;
        input_value = v;
        diagnostic = diag;
        return diagnostic;
    }
    eros::diagnostic set_commandvalue(double v)
    {
        eros::diagnostic diag = diagnostic;
        commandvalue_received_count++;
        command_value = v;
        diagnostic = diag;
        return diagnostic;
    }
    eros::diagnostic update(double dt);
    void reset_integral()
    {
        integral_error = 0;
        integral_reset_counter++;
    }
private:
    eros::signal initialize_signal(std::string name,uint8_t type);
    eros::diagnostic diagnostic;
     eros::controlgroup cg;
    Mode mode;
    bool initialized;
    bool ready;
    double run_time;
    eros::signal output;
    double gain_P;
    double gain_I;
    double gain_D;
    uint64_t inputvalue_received_count;
    uint64_t commandvalue_received_count;
    uint64_t integral_reset_counter;
    double input_value;
    double command_value;
    double error;
    double integral_error;
    double output_max;
    double output_min;
    double delta_output_limit;
   
};
#endif
