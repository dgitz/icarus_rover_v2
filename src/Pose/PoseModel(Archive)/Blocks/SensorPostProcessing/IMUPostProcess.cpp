/*
 * IMUPostProcess.cpp
 *
 *  Created on: Jan 2, 2019
 *      Author: robot
 */

#include "IMUPostProcess.h"

IMUPostProcess::IMUPostProcess() {
	// TODO Auto-generated constructor stub

}

IMUPostProcess::~IMUPostProcess() {
	// TODO Auto-generated destructor stub
}

PostProcessedSignal IMUPostProcess::new_signal(TimedSignal input)
{
	if(initialized == false)
	{
		PostProcessedSignal signal;
		signal.signal = input.signal;
		signal.signal_class = SIGNALCLASS_UNDEFINED;
		return signal;
	}
	else
	{
		PostProcessedSignal signal;
		signal.signal = input.signal;
		signal.signal_class = SIGNALCLASS_PROCESSEDSIGNAL;
		return signal;
	}
	
}