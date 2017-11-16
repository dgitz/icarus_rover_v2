#include "imu_node_process.h"
IMUNodeProcess::IMUNodeProcess()
{
	sensor_count = 0;
	message_ready = false;
	running = false;
	run_time = 0.0;
	delay_counter = 0;
	initialized = false;
}
IMUNodeProcess::~IMUNodeProcess()
{

}
icarus_rover_v2::diagnostic IMUNodeProcess::init(icarus_rover_v2::diagnostic indiag,std::string hostname)
{
    diagnostic = indiag;
	return diagnostic;
}
icarus_rover_v2::diagnostic IMUNodeProcess::update(double dt)
{
	run_time += dt;

	return diagnostic;
}
icarus_rover_v2::diagnostic IMUNodeProcess::new_devicemsg(icarus_rover_v2::device device)
{
	bool new_device = true;
	if(sensor.name == device.DeviceName)
	{
		new_device = false;
	}
	if(new_device == true)
	{
		if((device.PartNumber == "110012")) //Supported IMU list
		{
			Sensor s;
			s.ready = false;
			s.data_available = false;
			s.name = device.DeviceName;
			s.id = device.ID;
            s.tov = 0.0;
			s.buffer_size = 0;

            s.xacc_lastsum = 0.0;
            s.xacc_rms = -1.0;

            s.yacc_lastsum = 0.0;
            s.yacc_rms = -1.0;

            s.zacc_lastsum = 0.0;
            s.zacc_rms = -1.0;

            s.xgyro_lastsum = 0.0;
            s.xgyro_rms = -1.0;

            s.ygyro_lastsum = 0.0;
            s.ygyro_rms = -1.0;

            s.zgyro_lastsum = 0.0;
            s.zgyro_rms = -1.0;

            s.xmag_lastsum = 0.0;
            s.xmag_rms = -1.0;

            s.ymag_lastsum = 0.0;
            s.ymag_rms = -1.0;

            s.zmag_lastsum = 0.0;
            s.zmag_rms = -1.0;
			sensor = s;
			initialized = true;
			//initialize sensor here
		}
	}
	return diagnostic;
}
bool IMUNodeProcess::new_message(std::string msg)
{
	std::vector<std::string> strs;
	boost::split(strs,msg,boost::is_any_of(","));
	//printf("size: %d\n",strs.size());
	if(strs.size() == 12)
	{
		int id = std::atoi(strs.at(0).c_str());
		if(id == sensor.id)
		{
			try
			{
				sensor.last_tov = sensor.tov;
				sensor.tov = (double)(std::atoi(strs.at(1).c_str()));
				sensor.seq = std::atoi(strs.at(2).c_str());
				sensor.xacc = std::atof(strs.at(3).c_str());
				sensor.yacc = std::atof(strs.at(4).c_str());
				sensor.zacc = std::atof(strs.at(5).c_str());
				sensor.xgyro = std::atof(strs.at(6).c_str());
				sensor.ygyro = std::atof(strs.at(7).c_str());
				sensor.zgyro = std::atof(strs.at(8).c_str());
				sensor.xmag = std::atof(strs.at(9).c_str());
				sensor.ymag = std::atof(strs.at(10).c_str());
				sensor.zmag = std::atof(strs.at(11).c_str());
				sensor.ready = true;
				//sensor = compute_otherimudata(sensor);
			}
			catch(int e)
			{
				std::cout << "An error occurred at: IMUNodeProcess::new_message: " << e << std::endl;
			}
			double dt = sensor.tov - sensor.last_tov;
			if(dt > 12.0)
			{
				//printf("dt: %f\n",dt);
				delay_counter++;
			}
			sensor.data_available = true;
			return true;
		}
	}
	return false;
}
icarus_rover_v2::imu IMUNodeProcess::get_imudata(bool* valid,int id)
{
	icarus_rover_v2::imu data;
	if(id == sensor.id)
	{
		//icarus_rover_v2::imu data;
		if((sensor.ready == true) && (sensor.data_available == true))  //New sensor data
		{
			sensor.data_available = false;
			data.tov = sensor.tov;
			data.header.seq = sensor.seq;
			data.xacc.value = sensor.xacc;
			data.xacc.status = SIGNALSTATE_UPDATED;
			data.xacc.rms = -1.0;

			data.yacc.value = sensor.yacc;
			data.yacc.status = SIGNALSTATE_UPDATED;
			data.yacc.rms = -1.0;

			data.zacc.value = sensor.zacc;
			data.zacc.status = SIGNALSTATE_UPDATED;
			data.zacc.rms = -1.0;

			data.xgyro.value = sensor.xgyro;
			data.xgyro.status = SIGNALSTATE_UPDATED;
			data.xgyro.rms = -1.0;

			data.ygyro.value = sensor.ygyro;
			data.ygyro.status = SIGNALSTATE_UPDATED;
			data.ygyro.rms = -1.0;

			data.zgyro.value = sensor.zgyro;
			data.zgyro.status = SIGNALSTATE_UPDATED;
			data.zgyro.rms = -1.0;

			data.xmag.value = (double)sensor.xmag;
			data.xmag.status = SIGNALSTATE_UPDATED;
			data.xmag.rms = -1.0;

			data.ymag.value = (double)sensor.ymag;
			data.ymag.status = SIGNALSTATE_UPDATED;
			data.ymag.rms = -1.0;

			data.zmag.value = (double)sensor.zmag;
			data.zmag.status = SIGNALSTATE_UPDATED;
			data.zmag.rms = -1.0;
			*valid = true;
			return data;
		}
		else //No new sensor data
		{
			*valid = false;
			data.tov = -1.0;
			return data;
		}
		*valid = false;
		data.tov = -1.0;
		return data;
	}
	*valid = false;
	data.tov = -1.0;
	return data;
}
