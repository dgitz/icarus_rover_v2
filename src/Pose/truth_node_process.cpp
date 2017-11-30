#include "truth_node_process.h"
TruthNodeProcess::TruthNodeProcess()
{
	message_ready = false;
	running = false;
	run_time = 0.0;
	initialized = false;
	serialmessagehandler = new SerialMessageHandler;
	sensor.new_dataavailable = false;

    sensor.last_msgtime = 0.0;
}
TruthNodeProcess::~TruthNodeProcess()
{

}
icarus_rover_v2::diagnostic TruthNodeProcess::init(icarus_rover_v2::diagnostic indiag,std::string hostname_)
{
    diagnostic = indiag;
    hostname = hostname_;
	return diagnostic;
}
icarus_rover_v2::diagnostic TruthNodeProcess::update(double dt)
{
	run_time += dt;
	return diagnostic;
}
icarus_rover_v2::diagnostic TruthNodeProcess::new_devicemsg(icarus_rover_v2::device device)
{
    icarus_rover_v2::diagnostic diag = diagnostic;
	bool new_device = true;
	if(device.DeviceName == hostname)
	{
		
    }
	else
	{
		if((device.PartNumber == "810090")) //Supported IMU list
		{
			Sensor s;
			s.ready = false;
			s.name = device.DeviceName;
			s.id = device.ID;
			s.tov = 0.0;
			s.yawrate = 0.0;
			s.yaw = 0.0;
			s.pitchrate = 0.0;
			s.pitch = 0.0;
			s.rollrate = 0.0;
			s.roll = 0.0;
			s.north = 0.0;
			s.east = 0.0;
			s.elev = 0.0;
			s.wheelspeed = 0.0;
			s.groundspeed = 0.0;
			sensor = s;
			initialized = true;
			diag.Diagnostic_Type = SOFTWARE;
			diag.Level = INFO;
			char tempstr[512];
			sprintf(tempstr,"%s Initialized",sensor.name.c_str());
			diag.Description = std::string(tempstr);
			diag.Diagnostic_Message = NOERROR;
			return diag;
		}
		else
		{
            diag.Diagnostic_Type = SOFTWARE;
			diag.Level = ERROR;
			char tempstr[512];
			sprintf(tempstr,"%s Not Initialized",sensor.name.c_str());
			diag.Description = std::string(tempstr);
			diag.Diagnostic_Message = INITIALIZING_ERROR;
            return diag;
        }
	}
    return diag;
}
bool TruthNodeProcess::new_serialmessage(unsigned char* message,int length) //Return true: valid, false: invalid
{
	unsigned char uchar1;
	unsigned long ulong1;
	int int1;
	long long1,long2,long3,long4,long5,long6,long7,long8,long9;
	if(length > 4)
	{
		if(message[0] == 0xAB)
		{
			int id = message[1];
			int packet_length = (int)message[2];
			if((length-packet_length) == 5)
			{
				int computed_checksum = 0;
				for(int i = 3; i < length - 2; i++)
				{
					computed_checksum ^= message[i];
				}
				if(computed_checksum == message[length-2])
				{
					switch (id)
					{
						case SERIAL_ID_ID:
							serialmessagehandler->decode_IDSerial(message,&uchar1,&ulong1);
							break;
						default:
							printf("[TruthNode]: Message: 0XAB%0X Not Supported.\n",id);
							return false;
					}
					return true;
				}
			}
		}
		else
		{
			std::string tempstr;
						tempstr.append(reinterpret_cast<const char *>(message));
						std::vector<std::string> strs;
						boost::split(strs,tempstr,boost::is_any_of(","));
			if(strs.size() >= 10)
			{
				sensor.last_msgtime = run_time;
				sensor.last_tov = sensor.tov;
				sensor.tov = std::atof(strs.at(0).c_str());
				sensor.seq = std::atoi(strs.at(1).c_str());
				sensor.pitch = std::atof(strs.at(2).c_str());
				sensor.roll = std::atof(strs.at(3).c_str());
				sensor.yaw = std::atof(strs.at(4).c_str());
				sensor.east = std::atof(strs.at(5).c_str());
				sensor.north = std::atof(strs.at(6).c_str());
				sensor.elev = std::atof(strs.at(7).c_str());
				sensor.wheelspeed = std::atof(strs.at(8).c_str());
				sensor.groundspeed = std::atof(strs.at(9).c_str());
				sensor.ready = true;
				sensor.new_dataavailable = true;
				return true;
			}
			else
			{
				printf("[TruthNode:%s]: %s\n",sensor.name.c_str(),message);
			}
            	
		}
	}
	return false;
}

bool TruthNodeProcess::load_sensorinfo()
{
	std::string sensorinfo_path = "/home/robot/config/sensors/" + sensor.name + "/" + sensor.name + ".xml";
	TiXmlDocument doc(sensorinfo_path);
	bool loaded = doc.LoadFile();
	if(loaded == false)
	{
		printf("[%s]: Unable to load Sensor Config at: %s\n",sensor.name.c_str(),sensorinfo_path.c_str());
	}
	TiXmlElement *l_pRootElement = doc.RootElement();
	if(NULL != l_pRootElement)
	{

	}
	else { return false;}

	return true;
}
bool TruthNodeProcess::get_truthdata(icarus_rover_v2::pose *data)
{
	if((sensor.ready == true))
	{
		data->tov = sensor.tov;
		data->header.seq = sensor.seq;

		data->yawrate.units = "deg/s";
		data->yawrate.value = sensor.yawrate;
		data->yawrate.rms = -1.0;
		data->yawrate.status = SIGNALSTATE_INVALID;

		data->rollrate.units = "deg/s";
		data->rollrate.value = sensor.rollrate;
		data->rollrate.rms = -1.0;
		data->rollrate.status = SIGNALSTATE_INVALID;

		data->pitchrate.units = "deg/s";
		data->pitchrate.value = sensor.pitchrate;
		data->pitchrate.rms = -1.0;
		data->pitchrate.status = SIGNALSTATE_INVALID;

		data->yaw.units = "degree";
		data->yaw.value = sensor.yaw;
		data->yaw.rms = -1.0;

		data->roll.units = "degree";
		data->roll.value = sensor.roll;
		data->roll.rms = -1.0;

		data->pitch.units = "degree";
		data->pitch.value = sensor.pitch;
		data->pitch.rms = -1.0;

		data->east.units = "meter";
		data->east.value = sensor.east;
		data->east.rms = -1.0;

		data->north.units = "meter";
		data->north.value = sensor.north;
		data->north.rms = -1.0;

		data->elev.units = "meter";
		data->elev.value = sensor.elev;
		data->elev.rms = -1.0;

		data->wheelspeed.units = "m/s";
		data->wheelspeed.value = sensor.wheelspeed;
		data->wheelspeed.rms = -1.0;

		data->groundspeed.units = "m/s";
		data->groundspeed.value = sensor.groundspeed;
		data->groundspeed.rms = -1.0;

		if(sensor.new_dataavailable == true)
		{
			data->yawrate.status = SIGNALSTATE_UPDATED;
			data->pitchrate.status = SIGNALSTATE_UPDATED;
			data->rollrate.status = SIGNALSTATE_UPDATED;
			data->yaw.status = SIGNALSTATE_UPDATED;
			data->pitch.status = SIGNALSTATE_UPDATED;
			data->roll.status = SIGNALSTATE_UPDATED;
			data->east.status = SIGNALSTATE_UPDATED;
			data->north.status = SIGNALSTATE_UPDATED;
			data->elev.status = SIGNALSTATE_UPDATED;
			data->wheelspeed.status = SIGNALSTATE_UPDATED;
			data->groundspeed.status = SIGNALSTATE_UPDATED;
			sensor.new_dataavailable = false;
		}
		else
		{
			data->yawrate.status = SIGNALSTATE_HOLD;
			data->pitchrate.status = SIGNALSTATE_HOLD;
			data->rollrate.status = SIGNALSTATE_HOLD;
			data->yaw.status = SIGNALSTATE_HOLD;
			data->pitch.status = SIGNALSTATE_HOLD;
			data->roll.status = SIGNALSTATE_HOLD;
			data->east.status = SIGNALSTATE_HOLD;
			data->north.status = SIGNALSTATE_HOLD;
			data->elev.status = SIGNALSTATE_HOLD;
			data->wheelspeed.status = SIGNALSTATE_HOLD;
			data->groundspeed.status = SIGNALSTATE_HOLD;

		}
		return true;
	}
	else
	{
		return false;
	}
}
