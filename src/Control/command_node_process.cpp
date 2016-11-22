#include "command_node_process.h"

CommandNodeProcess::CommandNodeProcess()
{
	ms_timer = 0;
	timeout_value_ms = 0;
    init_time = ros::Time::now();
}
CommandNodeProcess::~CommandNodeProcess()
{

}
icarus_rover_v2::diagnostic CommandNodeProcess::init(icarus_rover_v2::diagnostic indiag,
		Logger *log,std::string hostname)
{
	//initialize_stateack_messages();
	myhostname = hostname;
	diagnostic = indiag;
	mylogger = log;
	mydevice.DeviceName = hostname;
	return diagnostic;
}
icarus_rover_v2::diagnostic CommandNodeProcess::update(long dt)
{
	ms_timer += dt;
	if(ms_timer >= timeout_value_ms) { timer_timeout = true; }
	if(timer_timeout == true)
	{
		timer_timeout = false;
		//printf("Mode: %d,%d\n",node_state,board_state);
		/*if((node_state == GPIO_MODE_INITIALIZING) && (board_state == GPIO_MODE_INITIALIZING))
		{
			//printf("Setting to true.\n");
			send_configure_DIO_PortA.trigger = true;
			send_configure_DIO_PortB.trigger = true;
			prev_node_state = node_state;
			node_state = GPIO_MODE_INITIALIZED;
		}
		*/

	}


	//send_nodemode.trigger = true;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = NOERROR;
	diagnostic.Description = "Node Executing.";
	return diagnostic;
}


icarus_rover_v2::diagnostic CommandNodeProcess::new_devicemsg(icarus_rover_v2::device newdevice)
{
	if((newdevice.DeviceName == myhostname) && (all_device_info_received == false))
	{
		mydevice = newdevice;

		all_device_info_received = true;
	}
	return diagnostic;
}
double CommandNodeProcess::time_diff(ros::Time timer_a, ros::Time timer_b)
{
	ros::Duration etime = timer_a - timer_b;
	return etime.toSec();
}
