#include "resourcemonitor.h"

ResourceMonitor::ResourceMonitor()
{
}
ResourceMonitor::ResourceMonitor(icarus_rover_v2::diagnostic diag,std::string device_architecture,std::string host_name,std::string task_name)
{  
	CPU_Used_Column = -1;
	RAM_Used_Column = -1;
	Device_Architecture = device_architecture;
	if(Device_Architecture == "x86_64")
	{
		CPU_Used_Column = 9;
		RAM_Used_Column = 5;
	}
	else if(Device_Architecture == "armv7l")
	{
		CPU_Used_Column = 9;
		RAM_Used_Column = 5;
	}

	Task_Name = task_name;
	Host_Name = host_name;
	generic_node_name = Task_Name.substr(Host_Name.length()+2,Task_Name.size());
	PID = -1;
	CPUUsed_perc = -1;
	RAMUsed_kB = -1;
	shortterm_buffer_index = 0;
	diagnostic = diag;
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
			if(fields.size() >= 8)
			{
				if(fields.at(0) == "%Cpu(s):")
				{
					CPUFree_perc =  (int)(atof(fields.at(7).c_str()));
					break;
				}
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
			if(fields.size() >= 2)
			{
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
	}
	myfile.close();
	if(Device_Architecture == "x86_64")
	{
		RAMFree_kB = memfree + memavail;
	}
	else if(Device_Architecture == "armv7l")
	{
		RAMFree_kB = memavail;
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
icarus_rover_v2::diagnostic ResourceMonitor::update()
{
	if((CPU_Used_Column == -1) || (RAM_Used_Column == -1))
	{
		diagnostic.Level = ERROR;
		diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
		diagnostic.Description = "Device Architecture not Supported.";
		return diagnostic;
	}
	if(PID == -1)
	{
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
			std::size_t found_node = line.find(find_string);
			std::size_t bad_find1 = line.find("sh -c");
			std::size_t bad_find2 = line.find("grep");
			if((found_node != std::string::npos) && (bad_find1 == std::string::npos) && (bad_find2 == std::string::npos))
			{
				std::vector <string> fields;
				boost::split(fields,line,boost::is_any_of("\t "),boost::token_compress_on);
				if(fields.size() >= 2)
				{
					id =  atoi(fields.at(1).c_str());
				}
			}

		}
		else
		{
			id = -1;
			diagnostic.Level = ERROR;
			diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
			char tempstr[255];
			sprintf(tempstr,"Unable to open PID File, trying to use: %s",pid_filename.c_str());
			diagnostic.Description = tempstr;
			return diagnostic;
		}
		myfile1.close();
		PID = id;
		if(id <= 0)
		{
			myfile1.open(pid_filename.c_str());
			if(myfile1.is_open())
			{
				std::string line;
				getline(myfile1,line);
				std::string find_string = generic_node_name;
				std::size_t found_node = line.find(find_string);
				std::size_t bad_find1 = line.find("sh -c");
				std::size_t bad_find2 = line.find("grep");
				if((found_node != std::string::npos) && (bad_find1 == std::string::npos) && (bad_find2 == std::string::npos))
				{
					printf("Found line: %s\n",line.c_str());
					std::vector <string> fields;
					boost::split(fields,line,boost::is_any_of("\t "),boost::token_compress_on);
					if(fields.size() >= 2)
					{
						//id =  atoi(fields.at(1).c_str());
					}
				}

			}
			myfile1.close();
			diagnostic.Level = ERROR;
			diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
			diagnostic.Description = "Unable to lookup Node PID";

			return diagnostic;
		}
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
		boost::split(strs,line,boost::is_any_of(" \t"),boost::token_compress_on);
		//for(int i = 0; i < strs.size();i++)
		//{
		//	printf("Task: %s i: %d s: %s\n",Task_Name.c_str(),i,strs.at(i).c_str());
		//}
		int size_required = std::max(CPU_Used_Column,RAM_Used_Column)+1;
		if(strs.size() >= size_required )
		{
			CPUUsed_perc = (int)(atof(strs.at(CPU_Used_Column).c_str()));
			RAMUsed_kB = atoi(strs.at(RAM_Used_Column).c_str());
			found = true;
		}

	}
	else
	{
		diagnostic.Level = ERROR;
		diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
		char tempstr[255];
		sprintf(tempstr,"Unable to open Resource File, trying to use: %s",resource_filename.c_str());
		diagnostic.Description = tempstr;
		return diagnostic;
	}
	myfile2.close();
	if(found == false)
	{
		diagnostic.Level = ERROR;
		diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
		char tempstr[128];
		sprintf(tempstr,"Unable to lookup Node Resources Used, Using PID: %d",PID);
		diagnostic.Description = tempstr;
		return diagnostic;
	}

	shortterm_buffer_RamUsed_kB.push_back(RAMUsed_kB);
	shortterm_buffer_index++;
	if(shortterm_buffer_RamUsed_kB.size() > SHORTTERM_BUFFER_SIZE)
	{
		shortterm_buffer_RamUsed_kB.erase(shortterm_buffer_RamUsed_kB.begin());
	}
	double sum = std::accumulate(shortterm_buffer_RamUsed_kB.begin(),shortterm_buffer_RamUsed_kB.end(),0.0);
	double mean = sum/shortterm_buffer_RamUsed_kB.size();
	//printf("Task: %s Found: %d Short Term avg RAM Used (kB): %f Size: %d\n",Task_Name.c_str(),found,mean,shortterm_buffer_RamUsed_kB.size());

	if(shortterm_buffer_index > SHORTTERM_BUFFER_SIZE)
	{
		shortterm_buffer_index = 0;
		longterm_buffer_RamUsed_kB.push_back(mean);
		if(longterm_buffer_RamUsed_kB.size() > LONGTERM_BUFFER_SIZE)
		{
			longterm_buffer_RamUsed_kB.erase(longterm_buffer_RamUsed_kB.begin());
		}
	}
	//printf("Task: %s Size: %d\n",
	//				Task_Name.c_str(),
	//				longterm_buffer_RamUsed_kB.size());
	if(longterm_buffer_RamUsed_kB.size() == LONGTERM_BUFFER_SIZE)
	{
		bool increasing = true;
		//int d_ramused_kb = longterm_buffer_RamUsed_kB.at(longterm_buffer_RamUsed_kB.size()-1) - longterm_buffer_RamUsed_kB.at(0);
		//if(d_ramused_kb <= 0)
		//{
		//	increasing = false;
		//}
		//printf("Task: %s d delta RAM Used: %d\n",Task_Name.c_str(),d_ramused_kb);
		int d_ramused_kb = 0;
		for(int i = longterm_buffer_RamUsed_kB.size()-6; i >= 0;i--)
		{
			d_ramused_kb = longterm_buffer_RamUsed_kB.at(longterm_buffer_RamUsed_kB.size()-1) - longterm_buffer_RamUsed_kB.at(i);
			if(d_ramused_kb <= 1)
			{
				increasing = false;
			}
			//printf("Task: %s d i: %d delta RAM Used: %d\n",Task_Name.c_str(),i,d_ramused_kb);
		}
		if(increasing == true)
		{
			if(d_ramused_kb > 1)
			{
				printf("Task: %s has a Memory leak! of approx: %d kB per sample\n",
						Task_Name.c_str(),
						d_ramused_kb);
			}
			if((d_ramused_kb > 10) && (d_ramused_kb <= 50))
			{
				diagnostic.Level = WARN;
				diagnostic.Diagnostic_Message = RESOURCE_LEAK;
				char tempstr[128];
				sprintf(tempstr,"Found RAM Leak: %f kB/s",(double)((double)d_ramused_kb/((double)SHORTTERM_BUFFER_SIZE*(double)longterm_buffer_RamUsed_kB.size())));
				diagnostic.Description = tempstr;
				return diagnostic;
			}
			else if(d_ramused_kb > 50)
			{
				diagnostic.Level = ERROR;
				diagnostic.Diagnostic_Message = RESOURCE_LEAK;
				char tempstr[256];
				sprintf(tempstr,"Found RAM Leak: %f kiloBytes per second",(double)((double)d_ramused_kb/((double)SHORTTERM_BUFFER_SIZE*(double)longterm_buffer_RamUsed_kB.size())));
				diagnostic.Description = tempstr;
				return diagnostic;
			}

		}


	}
	/*for(int i = 0; i < longterm_buffer_RamUsed_kB.size();i++)
	{

		printf("Task: %s i: %d Long Term avg RAM Used (kB): %f Size: %d\n",
				Task_Name.c_str(),i,
				longterm_buffer_RamUsed_kB.at(i),
				longterm_buffer_RamUsed_kB.size());
	}
	*/
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = NOERROR;
	diagnostic.Description = "Resource Usage Normal";
	return diagnostic;
}
