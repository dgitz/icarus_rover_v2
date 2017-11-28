#include "imu_node_process.h"
IMUNodeProcess::IMUNodeProcess()
{
	message_ready = false;
	running = false;
	run_time = 0.0;
	delay_counter = 0;
	initialized = false;
	serialmessagehandler = new SerialMessageHandler;
	sensor.new_dataavailable = false;
    sensor.buffer_size = 0;

    sensor.last_msgtime = 0.0;
    pitchangle_rad = 0.0;
    rollangle_rad = 0.0;
    yawangle_rad = 0.0;
    m_pitch = MatrixXd::Zero(3,3);
    m_roll = MatrixXd::Zero(3,3);
    m_yaw = MatrixXd::Zero(3,3);
    rotation_matrix = MatrixXd::Zero(3,3);
}
IMUNodeProcess::~IMUNodeProcess()
{

}
bool IMUNodeProcess::set_mountingangle(double pitch,double roll,double yaw)
{
    pitchangle_rad = pitch;
    rollangle_rad = roll;
    yawangle_rad = yaw;
    //Pitch: Ry
    //Roll: Rx
    //Yaw: Rz
    m_roll(0,0) = 1.0;
    m_roll(1,1) = cos(roll);
    m_roll(1,2) = -sin(roll);
    m_roll(2,1) = sin(roll);
    m_roll(2,2) = cos(roll);
    
    m_pitch(0,0) = cos(pitch);
    m_pitch(0,2) = sin(pitch);
    m_pitch(1,1) = 1.0;
    m_pitch(2,0) = -sin(pitch);
    m_pitch(2,2) = cos(pitch);
    
    m_yaw(0,0) = cos(yaw);
    m_yaw(0,1) = -sin(yaw);
    m_yaw(1,0) = sin(yaw);
    m_yaw(1,1) = cos(yaw);
    m_yaw(2,2) = 1.0;
    
    rotation_matrix = m_yaw*m_pitch*m_roll;
    printf("[%s] Using rotation matrix: \n",sensor.name.c_str());
    std::cout << rotation_matrix << std::endl;
    return true;
    
}
icarus_rover_v2::diagnostic IMUNodeProcess::init(icarus_rover_v2::diagnostic indiag,std::string hostname_)
{
    diagnostic = indiag;
    hostname = hostname_;
	return diagnostic;
}
icarus_rover_v2::diagnostic IMUNodeProcess::update(double dt)
{
	run_time += dt;
	if((run_time - sensor.last_msgtime) > 2.0)
	{
		printf("[%s]: delay: %f\n",sensor.name.c_str(),run_time-sensor.last_msgtime);
	}
	return diagnostic;
}
icarus_rover_v2::diagnostic IMUNodeProcess::new_devicemsg(icarus_rover_v2::device device)
{
    icarus_rover_v2::diagnostic diag = diagnostic;
	bool new_device = true;
	if(device.DeviceName == hostname)
	{
		
    }
	else
	{
		if((device.PartNumber == "110012")) //Supported IMU list
		{
			Sensor s;
			s.ready = false;
			s.name = device.DeviceName;
			s.id = device.ID;
            s.tov = 0.0;
			s.buffer_size = 0;

            s.xacc_lastsum = 0.0;
            s.xacc_rms = -1.0;
            s.xacc_bias = 0.0;

            s.yacc_lastsum = 0.0;
            s.yacc_rms = -1.0;
            s.yacc_bias = 0.0;

            s.zacc_lastsum = 0.0;
            s.zacc_rms = -1.0;
            s.zacc_bias = 0.0;

            s.xgyro_lastsum = 0.0;
            s.xgyro_rms = -1.0;
            s.xgyro_bias = 0.0;

            s.ygyro_lastsum = 0.0;
            s.ygyro_rms = -1.0;
            s.ygyro_bias = 0.0;

            s.zgyro_lastsum = 0.0;
            s.zgyro_rms = -1.0;
            s.zgyro_bias = 0.0;

            s.xmag_lastsum = 0.0;
            s.xmag_rms = -1.0;
            s.xmag_bias = 0.0;

            s.ymag_lastsum = 0.0;
            s.ymag_rms = -1.0;
            s.ymag_bias = 0.0;

            s.zmag_lastsum = 0.0;
            s.zmag_rms = -1.0;
            s.zmag_bias = 0.0;
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
bool IMUNodeProcess::new_serialmessage(unsigned char* message,int length) //Return true: valid, false: invalid
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
							printf("[IMUNode]: Message: 0XAB%0X Not Supported.\n",id);
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
			//tempstr = const_cast<std::string>(message);
			std::vector<std::string> strs;
			boost::split(strs,tempstr,boost::is_any_of(","));
            if(strs.size() >= 12)
            {
            	sensor.last_msgtime = run_time;
                sensor.last_tov = sensor.tov;
                sensor.tov = std::atof(strs.at(0).c_str());
                sensor.seq = std::atoi(strs.at(1).c_str());
                {
                    MatrixXd x = MatrixXd::Zero(3,1);
                    x(0,0) = (G*std::atof(strs.at(3).c_str()))-sensor.xacc_bias; //Forward
                    x(1,0) = (G*-1.0*std::atof(strs.at(2).c_str()))-sensor.yacc_bias; //Left
                    x(2,0) = (G*std::atof(strs.at(4).c_str()))-sensor.zacc_bias; //Up
                    x = rotation_matrix*x;
                    sensor.xacc = x(0,0);
                    sensor.yacc = x(1,0);
                    sensor.zacc = x(2,0);
                }
                {
                    MatrixXd x = MatrixXd::Zero(3,1);
                    x(0,0) = (std::atof(strs.at(5).c_str()))-sensor.xgyro_bias; //Roll, positive right
                    x(1,0) = (std::atof(strs.at(6).c_str()))-sensor.ygyro_bias; //Pitch, positive forward
                    x(2,0) = (std::atof(strs.at(7).c_str()))-sensor.zgyro_bias;  //Yaw, positive left
                    x = rotation_matrix*x;
                    sensor.xgyro = x(0,0);
                    sensor.ygyro = x(1,0);
                    sensor.zgyro = x(2,0);
                }
                {
                    MatrixXd x = MatrixXd::Zero(3,1);
                    x(0,0) = (std::atof(strs.at(8).c_str()))-sensor.xmag_bias;
                    x(1,0) = (std::atof(strs.at(9).c_str()))-sensor.ymag_bias;
                    x(2,0) = (std::atof(strs.at(10).c_str()))-sensor.zmag_bias;
                    sensor.xmag = x(0,0);
                    sensor.ymag = x(1,0);
                    sensor.zmag = x(2,0);
                }
                sensor.xacc_buffer.push_back(sensor.xacc);
                sensor.yacc_buffer.push_back(sensor.yacc);
                sensor.zacc_buffer.push_back(sensor.zacc);
                sensor.xgyro_buffer.push_back(sensor.xgyro);
                sensor.ygyro_buffer.push_back(sensor.ygyro);
                sensor.zgyro_buffer.push_back(sensor.zgyro);
                sensor.xmag_buffer.push_back(sensor.xmag);
                sensor.ymag_buffer.push_back(sensor.ymag);
                sensor.zmag_buffer.push_back(sensor.zmag);
                if(sensor.buffer_size > (RINGBUFFER_SIZE))
                {
                    
                }  
                else
                {
                    sensor.buffer_size++;
                }
                update_rms();

                
                sensor.ready = true;
                sensor.new_dataavailable = true;
                return true;
            }
            else
            {
            	
            }
			
		}

	}
	return false;
}
void IMUNodeProcess::update_rms()
{

    sensor.xacc_lastsum += pow(sensor.xacc,2.0);
    sensor.yacc_lastsum += pow(sensor.yacc,2.0);
    sensor.zacc_lastsum += pow(sensor.zacc,2.0);
    sensor.xgyro_lastsum += pow(sensor.xgyro,2.0);
    sensor.ygyro_lastsum += pow(sensor.ygyro,2.0);
    sensor.zgyro_lastsum +=  pow(sensor.zgyro,2.0);
    sensor.xmag_lastsum +=  pow(sensor.xmag,2.0);
    sensor.ymag_lastsum +=  pow(sensor.ymag,2.0);
    sensor.zmag_lastsum +=  pow(sensor.zmag,2.0);
    if(sensor.buffer_size > (RINGBUFFER_SIZE ))
    {
    	sensor.xacc_buffer.erase(sensor.xacc_buffer.begin());
    	sensor.yacc_buffer.erase(sensor.yacc_buffer.begin());
    	sensor.zacc_buffer.erase(sensor.zacc_buffer.begin());
    	sensor.xgyro_buffer.erase(sensor.xgyro_buffer.begin());
    	sensor.ygyro_buffer.erase(sensor.ygyro_buffer.begin());
    	sensor.zgyro_buffer.erase(sensor.zgyro_buffer.begin());
    	sensor.xmag_buffer.erase(sensor.xmag_buffer.begin());
    	sensor.ymag_buffer.erase(sensor.ymag_buffer.begin());
    	sensor.zmag_buffer.erase(sensor.zmag_buffer.begin());

    	sensor.xacc_lastsum -= pow(sensor.xacc_buffer.at(0),2.0);
    	sensor.yacc_lastsum -= pow(sensor.yacc_buffer.at(0),2.0);
    	sensor.zacc_lastsum -= pow(sensor.zacc_buffer.at(0),2.0);
    	sensor.xgyro_lastsum -= pow(sensor.xgyro_buffer.at(0),2.0);
    	sensor.ygyro_lastsum -= pow(sensor.ygyro_buffer.at(0),2.0);
    	sensor.zgyro_lastsum -= pow(sensor.zgyro_buffer.at(0),2.0);
    	sensor.xmag_lastsum -= pow(sensor.xmag_buffer.at(0),2.0);
    	sensor.ymag_lastsum -= pow(sensor.ymag_buffer.at(0),2.0);
    	sensor.zmag_lastsum -= pow(sensor.zmag_buffer.at(0),2.0);
    	//printf("here\n");



    }
    if(sensor.buffer_size > 2)
    {
    	sensor.xacc_rms = pow((1.0/(sensor.buffer_size-1))*sensor.xacc_lastsum,0.5);
    	sensor.yacc_rms = pow((1.0/(sensor.buffer_size-1))*sensor.yacc_lastsum,0.5);
    	sensor.zacc_rms = pow((1.0/(sensor.buffer_size-1))*sensor.zacc_lastsum,0.5);
    	sensor.xgyro_rms = pow((1.0/(sensor.buffer_size-1))*sensor.xgyro_lastsum,0.5);
    	sensor.ygyro_rms = pow((1.0/(sensor.buffer_size-1))*sensor.ygyro_lastsum,0.5);
    	sensor.zgyro_rms = pow((1.0/(sensor.buffer_size-1))*sensor.zgyro_lastsum,0.5);
    	sensor.xmag_rms = pow((1.0/(sensor.buffer_size-1))*sensor.xmag_lastsum,0.5);
    	sensor.ymag_rms = pow((1.0/(sensor.buffer_size-1))*sensor.ymag_lastsum,0.5);
    	sensor.zmag_rms = pow((1.0/(sensor.buffer_size-1))*sensor.zmag_lastsum,0.5);
    }
    else
    {
    	sensor.xacc_rms = -1.0;
    	sensor.yacc_rms = -1.0;
    	sensor.zacc_rms = -1.0;
    	sensor.xgyro_rms = -1.0;
    	sensor.ygyro_rms = -1.0;
    	sensor.zgyro_rms = -1.0;
    	sensor.xmag_rms = -1.0;
    	sensor.ymag_rms = -1.0;
    	sensor.zmag_rms = -1.0;
    }
    //printf("v: %f\n",sensor.xacc);
    //printf("%d %d %f %f %f\n",sensor.buffer_size,sensor.zmag_buffer.size(),sensor.zmag,sensor.zmag_lastsum,sensor.zmag_rms);
    

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
				std::cout << "[IMUNode]: An error occurred at: IMUNodeProcess::new_message: " << e << std::endl;
			}
			double dt = sensor.tov - sensor.last_tov;
			return true;
		}
	}
	return false;
}
icarus_rover_v2::imu IMUNodeProcess::get_imudata(bool* valid)
{
	icarus_rover_v2::imu data;
	//icarus_rover_v2::imu data;
	if((sensor.ready == true))// && (sensor.new_dataavailable == true))
	{
        data.tov = sensor.tov;
		data.header.seq = sensor.seq;

		data.xacc.units = "meter/s^2";
		data.xacc.value = sensor.xacc;
		data.xacc.rms = sensor.xacc_rms;

		data.yacc.units = "meter/s^2";
		data.yacc.value = sensor.yacc;
		data.yacc.rms = sensor.yacc_rms;

		data.zacc.units = "meter/s^2";
		data.zacc.value = sensor.zacc;
		data.zacc.rms = sensor.zacc_rms;

		data.xgyro.units = "deg/s";
		data.xgyro.value = sensor.xgyro;
		data.xgyro.rms = sensor.xgyro_rms;

		data.ygyro.units = "deg/s";
		data.ygyro.value = sensor.ygyro;
		data.ygyro.rms = sensor.ygyro_rms;

		data.zgyro.units = "deg/s";
		data.zgyro.value = sensor.zgyro;
		data.zgyro.rms = sensor.zgyro_rms;

		data.xmag.units = "uTesla";
		data.xmag.value = sensor.xmag;
		data.xmag.rms = sensor.xmag_rms;

		data.ymag.units = "uTesla";
		data.ymag.value = sensor.ymag;
		data.ymag.rms = sensor.ymag_rms;

		data.zmag.units = "uTesla";
		data.zmag.value = sensor.zmag;
		data.zmag.rms = sensor.zmag_rms;
        
        if(sensor.new_dataavailable == true)
        {
            data.xacc.status = SIGNALSTATE_UPDATED;
            data.yacc.status = SIGNALSTATE_UPDATED;
            data.zacc.status = SIGNALSTATE_UPDATED;
            data.xgyro.status = SIGNALSTATE_UPDATED;
            data.ygyro.status = SIGNALSTATE_UPDATED;
            data.zgyro.status = SIGNALSTATE_UPDATED;
            data.xmag.status = SIGNALSTATE_UPDATED;
            data.ymag.status = SIGNALSTATE_UPDATED;
            data.zmag.status = SIGNALSTATE_UPDATED;
            sensor.new_dataavailable = false;
        }
        else
        {
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
bool IMUNodeProcess::load_sensorinfo()
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
		TiXmlElement *l_pXAcc = l_pRootElement->FirstChildElement("XAcc");
		if(NULL != l_pXAcc)
		{
			TiXmlElement *l_pBias = l_pXAcc->FirstChildElement("Bias");
			if(NULL != l_pBias)
			{
				sensor.xacc_bias = std::atof(l_pBias->GetText());
			}
			else { return false; }
		}
		else { return false; }
		TiXmlElement *l_pYAcc = l_pRootElement->FirstChildElement("YAcc");
		if(NULL != l_pYAcc)
		{
			TiXmlElement *l_pBias = l_pYAcc->FirstChildElement("Bias");
			if(NULL != l_pBias)
			{
				sensor.yacc_bias = std::atof(l_pBias->GetText());
			}
			else { return false; }
		}
		else { return false; }
		TiXmlElement *l_pZAcc = l_pRootElement->FirstChildElement("ZAcc");
		if(NULL != l_pZAcc)
		{
			TiXmlElement *l_pBias = l_pZAcc->FirstChildElement("Bias");
			if(NULL != l_pBias)
			{
				sensor.zacc_bias = std::atof(l_pBias->GetText());
			}
			else { return false; }
		}
		else { return false; }
		TiXmlElement *l_pXGyro = l_pRootElement->FirstChildElement("XGyro");
		if(NULL != l_pXGyro)
		{
			TiXmlElement *l_pBias = l_pXGyro->FirstChildElement("Bias");
			if(NULL != l_pBias)
			{
				sensor.xgyro_bias = std::atof(l_pBias->GetText());
			}
			else { return false; }
		}
		else { return false; }
		TiXmlElement *l_pYGyro = l_pRootElement->FirstChildElement("YGyro");
		if(NULL != l_pYGyro)
		{
			TiXmlElement *l_pBias = l_pYGyro->FirstChildElement("Bias");
			if(NULL != l_pBias)
			{
				sensor.ygyro_bias = std::atof(l_pBias->GetText());
			}
			else { return false; }
		}
		else { return false; }
		TiXmlElement *l_pZGyro = l_pRootElement->FirstChildElement("ZGyro");
		if(NULL != l_pZGyro)
		{
			TiXmlElement *l_pBias = l_pZGyro->FirstChildElement("Bias");
			if(NULL != l_pBias)
			{
				sensor.zgyro_bias = std::atof(l_pBias->GetText());
			}
			else { return false; }
		}
		else { return false; }
		TiXmlElement *l_pXMag = l_pRootElement->FirstChildElement("XMag");
		if(NULL != l_pXMag)
		{
			TiXmlElement *l_pBias = l_pXMag->FirstChildElement("Bias");
			if(NULL != l_pBias)
			{
				sensor.xmag_bias = std::atof(l_pBias->GetText());
			}
			else { return false; }
		}
		else { return false; }
		TiXmlElement *l_pYMag = l_pRootElement->FirstChildElement("YMag");
		if(NULL != l_pYMag)
		{
			TiXmlElement *l_pBias = l_pYMag->FirstChildElement("Bias");
			if(NULL != l_pBias)
			{
				sensor.ymag_bias = std::atof(l_pBias->GetText());
			}
			else { return false; }
		}
		else { return false; }
		TiXmlElement *l_pZMag = l_pRootElement->FirstChildElement("ZMag");
		if(NULL != l_pZMag)
		{
			TiXmlElement *l_pBias = l_pZMag->FirstChildElement("Bias");
			if(NULL != l_pBias)
			{
				sensor.zmag_bias = std::atof(l_pBias->GetText());
						}
						else { return false; }
		}
		else { return false; }
	}
	else { return false;}

	return true;
}
