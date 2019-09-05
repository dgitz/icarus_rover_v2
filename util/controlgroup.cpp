#include "../include/controlgroup.h"

ControlGroup::ControlGroup()
{
}
ControlGroup::~ControlGroup()
{
}
eros::signal ControlGroup::initialize_signal(std::string name,uint8_t type)
{
	eros::signal sig;
	sig.value = 0.0;
	sig.name = name;
	sig.tov = -1.0;
	sig.type = type;
	sig.status = SIGNALSTATE_INITIALIZING;
	sig.rms = 0.0;
	return sig;
}
eros::diagnostic ControlGroup::init(eros::diagnostic t_diag,Mode t_mode)
{
	cg.input = initialize_signal("Input",SIGNALTYPE_UNDEFINED);
	cg.output = initialize_signal("Output",SIGNALTYPE_UNDEFINED);
	initialized = false;
	ready = false;
	run_time = 0.0;
	gain_P = 0.0;
    gain_I = 0.0;
    gain_D = 0.0;
	inputvalue_received_count = 0;
    commandvalue_received_count = 0;
    input_value = 0.0;
    command_value = 0.0;
	output.value = 0.0;
	error = 0.0;
	integral_error = 0.0;
	integral_reset_counter = 0;
	output_max = 0.0;
	output_min = 0.0;
	delta_output_limit = 0.0;
	diagnostic = t_diag;
	if(t_mode == ControlGroup::Mode::UNKNOWN)
	{
		diagnostic.Level = ERROR;
		diagnostic.Diagnostic_Type = SOFTWARE;
		diagnostic.Diagnostic_Message = INITIALIZING_ERROR;
		diagnostic.Description = "Unsupported Mode: UNKNOWN.";
		return diagnostic;
	}
	mode = t_mode;
	return diagnostic;
}
bool ControlGroup::set_PIDGains(double t_P,double t_I,double t_D)
{
	if(mode == ControlGroup::Mode::PID)
	{
		gain_P = t_P;
		gain_I = t_I;
		gain_D = t_D;
		initialized = true;
		return true;
	}
	else
	{
		return false;
	}
}
eros::diagnostic ControlGroup::update(double dt)
{
	eros::diagnostic diag = diagnostic;
	run_time+= dt;
	if(initialized == true)
	{
		if((inputvalue_received_count > 0) and (commandvalue_received_count > 0))
		{
			ready = true;
		}
	}
	if(ready == true)
	{
		double last_error = error;
		error = command_value-input_value;
		integral_error = (integral_error + error)*dt;
		double d_error = (error-last_error)/dt;
		double P_term = gain_P*error;
		double I_term = gain_I*integral_error;
		double D_term = gain_D*d_error;
		double v = (P_term + I_term + D_term);
		double d_out = (v-output.value)/dt;
		if(v > output_max)
		{
			diag.Level = WARN;
			diag.Diagnostic_Type = ACTUATORS;
			diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
			diag.Description = "Commanded Output of: "  + std::to_string(v) + " Exceeded Max Output Limit of: " + std::to_string(output_max);
			output.value = output_max;
		}
		else if(v < output_min)
		{
			diag.Level = WARN;
			diag.Diagnostic_Type = ACTUATORS;
			diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
			diag.Description = "Commanded Output of: "  + std::to_string(v) + " Exceeded Min Output Limit of: " + std::to_string(output_min);
			output.value = output_min;
		}
		else if(fabs(d_out) > delta_output_limit)
		{
			if(d_out > 0.0)
			{
				output.value = (delta_output_limit*dt)+output.value;
			}
			else
			{
				output.value = (-delta_output_limit*dt)+output.value;
			}
			diag.Level = WARN;
			diag.Diagnostic_Type = ACTUATORS;
			diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
			diag.Description = "Commanded Delta Output of: "  + std::to_string(d_out) + 
				" Exceeded Max Delta Output Limit of: " + std::to_string(delta_output_limit) + " Setting to: " + std::to_string(output.value);
			
		}
		else
		{
			output.value = v;
		}
	}
	diagnostic = diag;
	return diagnostic;
}