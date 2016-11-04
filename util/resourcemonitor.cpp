#include "resourcemonitor.h"

ResourceMonitor::ResourceMonitor()
{
}
ResourceMonitor::ResourceMonitor(std::string device_architecture,std::string host_name,std::string task_name)
{  
	CPU_Used_Column = -1;
	RAM_Used_Column = -1;
	Device_Architecture = device_architecture;
	if(Device_Architecture == "x86_64")
	{
		CPU_Used_Column = 9;
		RAM_Used_Column = 6;
	}
	else if(Device_Architecture == "armv7l")
	{
		CPU_Used_Column = 9;
		RAM_Used_Column = 6;
	}

	Task_Name = task_name;
	Host_Name = host_name;
	generic_node_name = Task_Name.substr(Host_Name.length()+2,Task_Name.size());
	PID = -1;
	CPUUsed_perc = -1;
	RAMUsed_kB = -1;
}
ResourceMonitor::~ResourceMonitor()
{
}
int ResourceMonitor::get_CPUFree_perc()
{
	std::string resource_filename;
	resource_filename = "/home/robot/logs/output/RESOURCE/top";
	char tempstr[130];
	sprintf(tempstr,"top -bn1 > %s",resource_filename.c_str());
	//printf("Command: %s\r\n",tempstr);
	system(tempstr); //RAM used is column 6 (RES), in KB.  CPU used is column 8, in percentage.
	ifstream myfile;
	myfile.open(resource_filename.c_str());
	if(myfile.is_open())
	{
		std::string line;
		while(getline(myfile,line))
		{
			//printf("Line:%s\r\n",line.c_str());
			std::vector <string> fields;
			boost::split(fields,line,boost::is_any_of("\t "),boost::token_compress_on);
			if(fields.at(0) == "%Cpu(s):")
			{
				CPUFree_perc =  (int)(atof(fields.at(7).c_str()));
				break;
			}
		}
	}
	myfile.close();
	return CPUFree_perc;
}
int ResourceMonitor::get_RAMFree_kB()
{
	ifstream myfile;
	myfile.open("/proc/meminfo");
	int memfree = 0;
	int memavail = 0;
	if(myfile.is_open())
	{
		std::string line;
		while(getline(myfile,line))
		{
			//printf("Line:%s\r\n",line.c_str());
			std::vector <string> fields;
			boost::split(fields,line,boost::is_any_of("\t "),boost::token_compress_on);
			if(fields.at(0) == "MemFree:")
			{
				memfree =  atoi(fields.at(1).c_str());
			}
			if(fields.at(0) == "MemAvailable:")
			{
				memavail =  atoi(fields.at(1).c_str());
			}

		}
	}
	myfile.close();
	if(Device_Architecture == "x86_64")
	{
		RAMFree_kB = memfree + memavail;
	}
	else if(Device_Architecture == "armv7l")
	{
		RAMFree_kB = memfree;
	}
	return RAMFree_kB;
}
int ResourceMonitor::get_CPUUsed_perc()
{
	return CPUUsed_perc;
}
int ResourceMonitor::get_RAMUsed_kB()
{
	return RAMUsed_kB;
}
int ResourceMonitor::get_TaskPID()
{
	return PID;
}
icarus_rover_v2::resource ResourceMonitor::get_resourceused()
{
	icarus_rover_v2::resource newresource;
	newresource.Node_Name = Task_Name;
	newresource.PID = PID;
	newresource.CPU_Perc = CPUUsed_perc;
	newresource.RAM_MB = (double)(RAMUsed_kB/1000.0);
	return newresource;
}
bool ResourceMonitor::update()
{
	if((CPU_Used_Column == -1) || (RAM_Used_Column == -1))
	{
		return false;
	}
	int id = -1;
	std::string local_node_name;
	local_node_name = Task_Name.substr(1,Task_Name.size());
	std::string pid_filename;
	pid_filename = "/home/robot/logs/output/PID" + Task_Name;
	char tempstr1[130];
	sprintf(tempstr1,"ps aux | grep __name:=%s > %s",local_node_name.c_str(),pid_filename.c_str());
	system(tempstr1);
	ifstream myfile1;
	myfile1.open(pid_filename.c_str());
	if(myfile1.is_open())
	{
		std::string line;
		getline(myfile1,line);
		std::string find_string = generic_node_name;
		std::size_t found = line.find(find_string);
		if(found != std::string::npos)
		{
			std::vector <string> fields;
			boost::split(fields,line,boost::is_any_of("\t "),boost::token_compress_on);
			id =  atoi(fields.at(1).c_str());
		}
	}
	else
	{
		id = -1;
	}
	myfile1.close();
	PID = id;
	if(id <= 0)
	{
		return false;
	}

	std::string resource_filename;
	resource_filename = "/home/robot/logs/output/RESOURCE/" + Task_Name;
	char tempstr2[130];
	sprintf(tempstr2,"top -bn1 | grep %d > %s",PID,resource_filename.c_str());
	//printf("Command: %s\r\n",tempstr);
	system(tempstr2); //RAM used is column 6 (RES), in KB.  CPU used is column 8, in percentage.
	ifstream myfile2;
	myfile2.open(resource_filename.c_str());
	bool found = false;
	if(myfile2.is_open())
	{
		std::string line;
		getline(myfile2,line);
		std::vector<std::string> strs;
		boost::split(strs,line,boost::is_any_of(" "),boost::token_compress_on);
		CPUUsed_perc = (int)(atof(strs.at(CPU_Used_Column).c_str()));
		RAMUsed_kB = atoi(strs.at(RAM_Used_Column).c_str());
		found = true;
	}
	else
	{
		found = false;
	}
	myfile2.close();
	return found;
}
