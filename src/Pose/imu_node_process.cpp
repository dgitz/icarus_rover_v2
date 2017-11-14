#include "imu_node_process.h"
IMUNodeProcess::IMUNodeProcess()
{
	sensor_count = 0;
	message_ready = false;
	running = false;
	run_time = 0.0;
	delay_counter = 0;
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
	for(std::size_t i = 0; i < sensors.size(); i++)
	{
		if(sensors.at(i).name == device.DeviceName)
		{
			new_device = false;
		}
	}
	if(new_device == true)
	{
		if((device.PartNumber == "110012")) //Supported IMU list
		{
			Sensor sensor;
			sensor.ready = false;
			sensor.data_available = false;
			sensor.name = device.DeviceName;
			sensor.id = device.ID;
			sensor.last_tov = 0.0;
			sensor.buffer_size = 0;
			sensor.xacc_buf.resize(RINGBUFFER_SIZE);

			sensors.push_back(sensor);
			//initialize sensor here
		}
	}
	return diagnostic;
}
bool IMUNodeProcess::new_message(std::string msg)
{
	std::vector<std::string> strs;
	boost::split(strs,msg,boost::is_any_of(","));
	if(strs.size() == 11)
	{
		int id = std::atoi(strs.at(0).c_str());
		bool found = false;
		for(std::size_t i = 0; i < sensors.size(); i++)
		{
			if(id == sensors.at(i).id)
			{
				found = true;

				sensors.at(i).tov = std::atof(strs.at(1).c_str());
				sensors.at(i).xacc = std::atof(strs.at(2).c_str());
				sensors.at(i).yacc = std::atof(strs.at(3).c_str());
				sensors.at(i).zacc = std::atof(strs.at(4).c_str());
				sensors.at(i).xgyro = std::atof(strs.at(5).c_str());
				sensors.at(i).ygyro = std::atof(strs.at(6).c_str());
				sensors.at(i).zgyro = std::atof(strs.at(7).c_str());
				sensors.at(i).xmag = std::atof(strs.at(8).c_str());
				sensors.at(i).ymag = std::atof(strs.at(9).c_str());
				sensors.at(i).zmag = std::atof(strs.at(10).c_str());
				sensors.at(i) = compute_otherimudata(sensors.at(i));
				double dt = sensors.at(i).tov - sensors.at(i).last_tov;
				if(dt > 12.0)
				{
					delay_counter++;
				}
				sensors.at(i).data_available = true;
				return true;
			}
		}
		return false;
	}
	return false;
}
IMUNodeProcess::Sensor IMUNodeProcess::compute_otherimudata(Sensor sensor)
{
	sensor.xacc_buf.push_back(sensor.xacc);
	sensor.yacc_buf.push_back(sensor.yacc);
	sensor.zacc_buf.push_back(sensor.zacc);
	sensor.xgyro_buf.push_back(sensor.xgyro);
	sensor.ygyro_buf.push_back(sensor.ygyro);
	sensor.zgyro_buf.push_back(sensor.zgyro);
	sensor.xmag_buf.push_back(sensor.xmag);
	sensor.ymag_buf.push_back(sensor.ymag);
	sensor.ymag_buf.push_back(sensor.zmag);
	sensor.buffer_size++;
	if(sensor.buffer_size > RINGBUFFER_SIZE)
	{
		sensor.xacc_rms = compute_rms(sensor.xacc_buf);
		sensor.yacc_rms = compute_rms(sensor.yacc_buf);
		sensor.zacc_rms = compute_rms(sensor.zacc_buf);
		sensor.xgyro_rms = compute_rms(sensor.xgyro_buf);
		sensor.ygyro_rms = compute_rms(sensor.ygyro_buf);
		sensor.zgyro_rms = compute_rms(sensor.zgyro_buf);
		sensor.xmag_rms = compute_rms(sensor.xmag_buf);
		sensor.ymag_rms = compute_rms(sensor.ymag_buf);
		sensor.zmag_rms = compute_rms(sensor.zmag_buf);
		sensor.ready = true;
		sensor.buffer_size = RINGBUFFER_SIZE;
	}
	return sensor;
}
bool IMUNodeProcess::imudata_ready(int id)
{
	for(std::size_t i = 0; i < sensors.size(); i++)
	{
		if(sensors.at(i).id == id)
		{
			return sensors.at(i).data_available;
		}
	}
	return false;
}
icarus_rover_v2::imu IMUNodeProcess::get_imudata(int id)
{
	for(std::size_t i = 0; i < sensors.size(); i++)
	{
		if(sensors.at(i).id == id)
		{
			icarus_rover_v2::imu data;
			if(sensors.at(i).ready == true)
			{
				data.tov = sensors.at(i).tov;
				data.xacc.value = sensors.at(i).xacc;
				data.xacc.status = SIGNALSTATE_UPDATED;
				data.xacc.rms = sensors.at(i).xacc_rms;

				data.yacc.value = sensors.at(i).yacc;
				data.yacc.status = SIGNALSTATE_UPDATED;
				data.yacc.rms = sensors.at(i).yacc_rms;

				data.zacc.value = sensors.at(i).zacc;
				data.zacc.status = SIGNALSTATE_UPDATED;
				data.zacc.rms = sensors.at(i).zacc_rms;

				data.xgyro.value = sensors.at(i).xgyro;
				data.xgyro.status = SIGNALSTATE_UPDATED;
				data.xgyro.rms = sensors.at(i).xgyro_rms;

				data.ygyro.value = sensors.at(i).ygyro;
				data.ygyro.status = SIGNALSTATE_UPDATED;
				data.ygyro.rms = sensors.at(i).ygyro_rms;

				data.zgyro.value = sensors.at(i).zgyro;
				data.zgyro.status = SIGNALSTATE_UPDATED;
				data.zgyro.rms = sensors.at(i).zgyro_rms;

				data.xmag.value = sensors.at(i).xmag;
				data.xmag.status = SIGNALSTATE_UPDATED;
				data.xmag.rms = sensors.at(i).xmag_rms;

				data.ymag.value = sensors.at(i).ymag;
				data.ymag.status = SIGNALSTATE_UPDATED;
				data.ymag.rms = sensors.at(i).ymag_rms;

				data.zmag.value = sensors.at(i).zmag;
				data.zmag.status = SIGNALSTATE_UPDATED;
				data.zmag.rms = sensors.at(i).zmag_rms;

			}
			else
			{
				data.tov = sensors.at(i).tov;
				data.xacc.status = SIGNALSTATE_HOLD;
				data.yacc.status = SIGNALSTATE_HOLD;
				data.zacc.status = SIGNALSTATE_HOLD;
				data.xgyro.status = SIGNALSTATE_HOLD;
				data.ygyro.status = SIGNALSTATE_HOLD;
				data.zgyro.status = SIGNALSTATE_HOLD;
				data.xmag.status = SIGNALSTATE_HOLD;
				data.ymag.status = SIGNALSTATE_HOLD;
				data.zmag.status = SIGNALSTATE_HOLD;
			}
			return data;
		}
	}
	icarus_rover_v2::imu empty;
	empty.tov = -1.0;
	return empty;
}
double IMUNodeProcess::compute_rms(boost::circular_buffer<double> buf)
{
	double sum = 0.0;
	for(std::size_t i = 0; i < buf.size(); i++)
	{
		sum += buf.at(i)*buf.at(i);
	}
	sum = sum/buf.size();
	sum = pow(sum,0.5);
	return sum;
}
double IMUNodeProcess::compute_rms(boost::circular_buffer<int> buf)
{
	double sum = 0.0;
	for(std::size_t i = 0; i < buf.size(); i++)
	{
		sum += (double)(buf.at(i))*(double)(buf.at(i));
	}
	sum = sum/buf.size();
	sum = pow(sum,0.5);
}
