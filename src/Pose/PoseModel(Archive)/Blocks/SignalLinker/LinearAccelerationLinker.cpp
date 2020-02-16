/*
 * LinearAccelerationLinker.cpp
 *
 *  Created on: Jan 2, 2019
 *      Author: robot
 */

#include "LinearAccelerationLinker.h"
LinearAccelerationLinker::LinearAccelerationLinker() {
	// TODO Auto-generated constructor stub

}

LinearAccelerationLinker::~LinearAccelerationLinker() {
	// TODO Auto-generated destructor stub
}
eros::diagnostic LinearAccelerationLinker::new_input(std::vector<SplitSignal> inputs)
{
    eros::diagnostic diag = diagnostic;
    if(initialized == false)
    {
        for(std::size_t i = 0; i < inputs.size(); ++i)
        {
            InputSignal_3d output;
            output.instance_name = std::to_string(i);
            output.x = inputs.at(i).x;
            output.y = inputs.at(i).y;
            output.z = inputs.at(i).z;
            output.x_update_count = 0;
            output.y_update_count = 0;
            output.z_update_count = 0;
            outputs.push_back(output);
        }
        input_count = (int)inputs.size();
        initialized = true;
    }
    diag = update_input(inputs);
    diagnostic = diag;
    return diagnostic;
}
eros::diagnostic LinearAccelerationLinker::update_input(std::vector<SplitSignal> inputs)
{
    eros::diagnostic diag = diagnostic;
    for(std::size_t i = 0; i < inputs.size(); ++i)
    {
        outputs.at(i).x = inputs.at(i).x;
        outputs.at(i).y = inputs.at(i).y;
        outputs.at(i).z = inputs.at(i).z;
        outputs.at(i).x_update_count++;
        outputs.at(i).y_update_count++;
        outputs.at(i).z_update_count++;
    }
    diagnostic = diag;
    return diagnostic;
}