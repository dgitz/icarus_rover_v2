/*
 * MasterLinker.cpp
 *
 *  Created on: Jan 2, 2019
 *      Author: robot
 */

#include "SignalSplitter.h"

SignalSplitter::SignalSplitter() 
{
    initialized = false;
}
SignalSplitter::~SignalSplitter() 
{
}
eros::diagnostic SignalSplitter::new_input(std::vector<PostProcessedSignal> signals)
{
    eros::diagnostic diag = diagnostic;
    if(initialized == false)
    {
        diag = init_signals(signals);
        initialized = true;
    }
    diag = update_input(signals);
    diagnostic = diag;
    return diagnostic;
}
eros::diagnostic SignalSplitter::init_signals(std::vector<PostProcessedSignal> signals)
{
    eros::diagnostic diag = diagnostic;
    std::string error_string = "";
    uint16_t error_count = 0;
    for(std::size_t i = 0; i < signals.size(); ++i)
    {    
        bool processed_signal_name = false;
        if( (signals.at(i).signal.name.find("xacc") != std::string::npos) || 
            (signals.at(i).signal.name.find("yacc") != std::string::npos) || 
            (signals.at(i).signal.name.find("zacc") != std::string::npos))
        {
            processed_signal_name = true;
            std::string instance_name = signals.at(i).signal.name.substr(4);
            bool found_instance = false;
            for(std::size_t j = 0; j < split_linearaccelerations.size(); ++j)
            {
                if(split_linearaccelerations.at(j).instance_name == instance_name)
                {
                    found_instance = true;
                }
            }
            if(found_instance == false)
            {
                SplitSignal acc;
                acc.instance_name = instance_name;
                acc.x.name = "xacc" + instance_name;
                acc.y.name = "yacc" + instance_name;
                acc.z.name = "zacc" + instance_name;
                acc.x_update_count = 0;
                acc.y_update_count = 0;
                acc.z_update_count = 0;
                split_linearaccelerations.push_back(acc);
            }
        }
        if( (signals.at(i).signal.name.find("xgyro") != std::string::npos) || 
            (signals.at(i).signal.name.find("ygyro") != std::string::npos) || 
            (signals.at(i).signal.name.find("zgyro") != std::string::npos))
        {
            processed_signal_name = true;
            std::string instance_name = signals.at(i).signal.name.substr(5);
            bool found_instance = false;
            for(std::size_t j = 0; j < split_rotationrates.size(); ++j)
            {
                if(split_rotationrates.at(j).instance_name == instance_name)
                {
                    found_instance = true;
                }
            }
            if(found_instance == false)
            {
                SplitSignal rot;
                rot.instance_name = instance_name;
                rot.x.name = "xgyro" + instance_name;
                rot.y.name = "ygyro" + instance_name;
                rot.z.name = "zgyro" + instance_name;
                rot.x_update_count = 0;
                rot.y_update_count = 0;
                rot.z_update_count = 0;
                split_rotationrates.push_back(rot);
            }
        }
        if( (signals.at(i).signal.name.find("xmag") != std::string::npos) || 
            (signals.at(i).signal.name.find("ymag") != std::string::npos) || 
            (signals.at(i).signal.name.find("zmag") != std::string::npos))
        {
            processed_signal_name = true;
            std::string instance_name = signals.at(i).signal.name.substr(4);
            bool found_instance = false;
            for(std::size_t j = 0; j < split_magneticfields.size(); ++j)
            {
                if(split_magneticfields.at(j).instance_name == instance_name)
                {
                    found_instance = true;
                }
            }
            if(found_instance == false)
            {
                SplitSignal mag;
                mag.instance_name = instance_name;
                mag.x.name = "xmag" + instance_name;
                mag.y.name = "ymag" + instance_name;
                mag.z.name = "zmag" + instance_name;
                mag.x_update_count = 0;
                mag.y_update_count = 0;
                mag.z_update_count = 0;
                split_magneticfields.push_back(mag);
            }
        }
        if(processed_signal_name == false)
        {
            error_string += "ERROR: Unable to split Signal: " + signals.at(i).signal.name;
            error_count++;
            printf("Unable to split signal: %s\n",signals.at(i).signal.name.c_str());
        }
    }
    if(error_count == 0)
    {
        diag.Level = INFO;
        diag.Diagnostic_Message = NOERROR;
        diag.Description = "Initialized.";
    }
    else
    {
        diag.Level = ERROR;
        diag.Diagnostic_Message = INITIALIZING_ERROR;
        diag.Description = error_string;
    }
    
    diagnostic = diag;
    return diag;
}
eros::diagnostic SignalSplitter::update_input(std::vector<PostProcessedSignal> signals)
{
    eros::diagnostic diag = diagnostic;
    for(std::size_t i =0; i < signals.size(); ++i)
    {
        bool processed_signal_name = false;
        for(std::size_t j = 0; j < split_linearaccelerations.size(); ++j)
        {
            if(split_linearaccelerations.at(j).x.name == signals.at(i).signal.name)
            {
                processed_signal_name = true;
                split_linearaccelerations.at(j).x = signals.at(i).signal;
                split_linearaccelerations.at(j).x_update_count++;
                break;
            }
            if(split_linearaccelerations.at(j).y.name == signals.at(i).signal.name)
            {
                processed_signal_name = true;
                split_linearaccelerations.at(j).y = signals.at(i).signal;
                split_linearaccelerations.at(j).y_update_count++;
                break;
            }
            if(split_linearaccelerations.at(j).z.name == signals.at(i).signal.name)
            {
                processed_signal_name = true;
                split_linearaccelerations.at(j).z = signals.at(i).signal;
                split_linearaccelerations.at(j).z_update_count++;
                break;
            }
        }
        for(std::size_t j = 0; j < split_rotationrates.size(); ++j)
        {
            if(split_rotationrates.at(j).x.name == signals.at(i).signal.name)
            {
                processed_signal_name = true;
                split_rotationrates.at(j).x = signals.at(i).signal;
                split_rotationrates.at(j).x_update_count++;
                break;
            }
            if(split_rotationrates.at(j).y.name == signals.at(i).signal.name)
            {
                processed_signal_name = true;
                split_rotationrates.at(j).y = signals.at(i).signal;
                split_rotationrates.at(j).y_update_count++;
                break;
            }
            if(split_linearaccelerations.at(j).z.name == signals.at(i).signal.name)
            {
                processed_signal_name = true;
                split_rotationrates.at(j).z = signals.at(i).signal;
                split_rotationrates.at(j).z_update_count++;
                break;
            }
        }
        for(std::size_t j = 0; j < split_magneticfields.size(); ++j)
        {
            if(split_magneticfields.at(j).x.name == signals.at(i).signal.name)
            {
                processed_signal_name = true;
                split_magneticfields.at(j).x = signals.at(i).signal;
                split_magneticfields.at(j).x_update_count++;
                break;
            }
            if(split_magneticfields.at(j).y.name == signals.at(i).signal.name)
            {
                processed_signal_name = true;
                split_magneticfields.at(j).y = signals.at(i).signal;
                split_magneticfields.at(j).y_update_count++;
                break;
            }
            if(split_magneticfields.at(j).z.name == signals.at(i).signal.name)
            {
                processed_signal_name = true;
                split_magneticfields.at(j).z = signals.at(i).signal;
                split_magneticfields.at(j).z_update_count++;
                break;
            }
        }
    }
    diag.Diagnostic_Message = NOERROR;
    diag.Level = INFO;
    diag.Description = "SignalSplitter Updated.";
    diagnostic = diag;
    return diag;
}
void SignalSplitter::print_splitsignals()
{
    printf("--- SPLIT SIGNALS: ACCELERATION ---\n");
    for(std::size_t i = 0; i < split_linearaccelerations.size(); ++i)
    {
        printf("\t[%d/%d] Instance: %s X Update Count: %ld Y Update Count: %ld Z Update Count: %ld\n",
            (int)i+1,(int)split_linearaccelerations.size(),split_linearaccelerations.at(i).instance_name.c_str(),
            split_linearaccelerations.at(i).x_update_count,
            split_linearaccelerations.at(i).y_update_count,
            split_linearaccelerations.at(i).z_update_count);

    }
    printf("--- END SPLIT SIGNALS: ACCELERATION ---\n");
    printf("--- SPLIT SIGNALS: ROTATION RATE ---\n");
    for(std::size_t i = 0; i < split_rotationrates.size(); ++i)
    {
        printf("\t[%d/%d] Instance: %s X Update Count: %ld Y Update Count: %ld Z Update Count: %ld\n",
            (int)i+1,(int)split_rotationrates.size(),split_rotationrates.at(i).instance_name.c_str(),
            split_rotationrates.at(i).x_update_count,
            split_rotationrates.at(i).y_update_count,
            split_rotationrates.at(i).z_update_count);

    }
    printf("--- END SPLIT SIGNALS: ROTATION RATE ---\n");
    printf("--- SPLIT SIGNALS: MAGNETIC FIELD ---\n");
    for(std::size_t i = 0; i < split_magneticfields.size(); ++i)
    {
        printf("\t[%d/%d] Instance: %s X Update Count: %ld Y Update Count: %ld Z Update Count: %ld\n",
            (int)i+1,(int)split_magneticfields.size(),split_magneticfields.at(i).instance_name.c_str(),
            split_magneticfields.at(i).x_update_count,
            split_magneticfields.at(i).y_update_count,
            split_magneticfields.at(i).z_update_count);

    }
    printf("--- END SPLIT SIGNALS: MAGNETIC FIELD ---\n");
}
