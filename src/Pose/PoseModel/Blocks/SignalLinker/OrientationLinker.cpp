/*
 * OrientationLinker.cpp
 *
 *  Created on: Jan 2, 2019
 *      Author: robot
 */

#include "OrientationLinker.h"
OrientationLinker::OrientationLinker()
{
    // TODO Auto-generated constructor stub
}

OrientationLinker::~OrientationLinker()
{
    // TODO Auto-generated destructor stub
}
eros::diagnostic OrientationLinker::new_input(std::vector<std::vector<InputSignal_3d>> inputs_vector)
{
    eros::diagnostic diag = diagnostic;
    if (initialized == false)
    {
        for (std::size_t i = 0; i < inputs_vector.size(); ++i)
        {
            if (i == (uint8_t)Pipeline::LINEARACCELERATION)
            {
                for (std::size_t j = 0; j < inputs_vector.at(i).size(); ++j)
                {
                    eros::signal x;
                    eros::signal y;
                    eros::signal z;
                    x.name = "roll" + std::to_string(i) + "_" + std::to_string(j);
                    x.type = SIGNALTYPE_ANGLE;
                    x.status = SIGNALSTATE_INITIALIZING;
                    y.name = "pitch" + std::to_string(i) + "_" + std::to_string(j);
                    y.type = SIGNALTYPE_ANGLE;
                    y.status = SIGNALSTATE_INITIALIZING;
                    z.name = "yaw" + std::to_string(i) + "_" + std::to_string(j);
                    z.type = SIGNALTYPE_ANGLE;
                    z.status = SIGNALSTATE_INVALID; // Cannot compute Yaw Orientation from Acceleration Data
                    InputSignal_3d output;
                    output.instance_name = std::to_string(i) + "_" + std::to_string(j);
                    output.x = x;
                    output.y = y;
                    output.z = z;
                    output.x_update_count = 0;
                    output.y_update_count = 0;
                    output.z_update_count = 0;
                    outputs.push_back(output);
                }
            }
            else
            {
                diag.Level = ERROR;
                diag.Diagnostic_Message = INITIALIZING_ERROR;
                diag.Description = "OrientationLinker: Pipeline: " + std::to_string(i) + " is Not Supported.";
                return diag;
            }
        }
        initialized = true;
    }
    diag = update_input(inputs_vector);
    diagnostic = diag;
    return diagnostic;
}
eros::diagnostic OrientationLinker::update_input(std::vector<std::vector<InputSignal_3d>> inputs_vector)
{
    eros::diagnostic diag = diagnostic;
    uint16_t pipeline_index = 0;
    for (std::size_t i = 0; i < inputs_vector.size(); ++i)
    {
        if (i == (uint8_t)Pipeline::LINEARACCELERATION)
        {
            for (std::size_t j = 0; j < inputs_vector.at(i).size(); ++j)
            {
                double acc_x = inputs_vector.at(i).at(j).x.value;
                double acc_y = inputs_vector.at(i).at(j).y.value;
                double acc_z = inputs_vector.at(i).at(j).z.value;
                double theta = atan2(-acc_x, pow(((acc_y * acc_y) + (acc_z * acc_z)), 0.5));
                double sign_acc_z = 1.0;
                if (acc_z < 0.0)
                {
                    sign_acc_z = -1.0;
                }
                double phi = atan2(acc_y, (sign_acc_z * pow(((acc_z * acc_z) + LINEARACC_MU * (acc_x * acc_x)), 0.5)));
                outputs.at(pipeline_index).x.tov = inputs_vector.at(i).at(j).x.tov;
                outputs.at(pipeline_index).x.status = inputs_vector.at(i).at(j).x.status;
                outputs.at(pipeline_index).y.value = phi * 180.0 / M_PI;
                outputs.at(pipeline_index).y.tov = inputs_vector.at(i).at(j).y.tov;
                outputs.at(pipeline_index).y.status = inputs_vector.at(i).at(j).y.status;
                outputs.at(pipeline_index).y.value = theta * 180.0 / M_PI;
                outputs.at(pipeline_index).x_update_count++;
                outputs.at(pipeline_index).y_update_count++;
                pipeline_index++;
            }
        }
        else
        {
            diag.Level = ERROR;
            diag.Diagnostic_Message = INITIALIZING_ERROR;
            diag.Description = "OrientationLinker: Pipeline: " + std::to_string(i) + " is Not Supported.";
            return diag;
        }
    }
    diagnostic = diag;
    return diagnostic;
}