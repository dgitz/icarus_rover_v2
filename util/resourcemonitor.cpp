#include "../include/resourcemonitor.h"

ResourceMonitor::ResourceMonitor()
{
}
ResourceMonitor::ResourceMonitor(eros::diagnostic diag,std::string device_architecture,std::string host_name,std::string task_name)
{  
	init(diag,device_architecture,host_name,task_name);
}
void ResourceMonitor::init(eros::diagnostic diag,std::string device_architecture,std::string host_name,std::string task_name)
{
	ramfree_initialized = false;
	memoryleak_detectection_enabled = true;
	resourceusage_diagnostic_enabled = true;
	Device_Architecture = device_architecture;
	Task_Name = task_name;
	Host_Name = host_name;
	generic_node_name = Task_Name;
	PID = -1;
	CPUUsed_perc = -1;
	RAMUsed_kB = -1;
	shortterm_buffer_index = 0;
	diagnostic = diag;
	diagnostic.Diagnostic_Type = SYSTEM_RESOURCE;
	std::string tempstr = exec("nproc",true);
	boost::trim_right(tempstr);
	processor_count = std::atoi(tempstr.c_str());


	diagnostic = update();
}
ResourceMonitor::~ResourceMonitor()
{
}
int ResourceMonitor::get_RAMFree_perc()
{
	if(ramfree_initialized == false)
	{
		std::ifstream file( "/proc/meminfo" );
		if(!file)
		{
			diagnostic.Level = ERROR;
			diagnostic.Diagnostic_Message = DROPPING_PACKETS;
			diagnostic.Description = "Unable to read /proc/meminfo";
		}
		int found_entry_count = 0;
		for( std::string line; getline( file, line ); )
		{
			std::vector <string> fields;
			boost::split(fields,line,boost::is_any_of("\t "),boost::token_compress_on);
			std::string value = "";
			if(fields.size() != 3)
			{
				continue;
			}
			if(std::string::npos == fields.at(2).find("kB"))
			{
				continue;
			}
			if(std::string::npos != fields.at(0).find("MemTotal:"))
			{
				RAMTotal_kb = std::atoi(fields.at(1).c_str());
				found_entry_count++;
			}
		}
		file.close();
		if(found_entry_count == 0)
		{
			diagnostic.Level = ERROR;
			diagnostic.Diagnostic_Message = DROPPING_PACKETS;
			diagnostic.Description = "Unable to process /proc/meminfo";
		}
		else
		{
			ramfree_initialized = true;
		}
	}
	double num = (double)get_RAMFree_kB();
	double den = (double)RAMTotal_kb;
	return (int)(100.0*num/den);
}
int ResourceMonitor::get_CPUFree_perc()
{
	std::string tempstr = exec("top -bn2 -d1 | grep \"Cpu(s)\" | tail -1",true);
	std::vector <string> fields;
	boost::split(fields,tempstr,boost::is_any_of("\t "),boost::token_compress_on);
	if(fields.size() < 9)
	{
		diagnostic.Level = ERROR;
		diagnostic.Diagnostic_Message = DROPPING_PACKETS;
		diagnostic.Description = "Unable to process Free CPU.";
		CPUFree_perc = -1;
		return CPUFree_perc;
	}

	if(std::string::npos != fields.at(8).find("id,"))
	{
		CPUFree_perc = (int)(std::atof(fields.at(7).c_str()));
		return CPUFree_perc;
	}
	else
	{
		diagnostic.Level = ERROR;
		diagnostic.Diagnostic_Message = DROPPING_PACKETS;
		diagnostic.Description = "Unable to process Free CPU.";
		CPUFree_perc = -1;
		return CPUFree_perc;
	}
}
int ResourceMonitor::get_RAMFree_kB()
{
	int memfree = 0;
	int memavail = 0;
    int membuffer = 0;
    int memcached = 0;
	std::ifstream file( "/proc/meminfo" );
	if(!file)
	{
		diagnostic.Level = ERROR;
		diagnostic.Diagnostic_Message = DROPPING_PACKETS;
		diagnostic.Description = "Unable to read /proc/meminfo";
		RAMFree_kB = -1;
		return RAMFree_kB;
	}
	int found_entry_count = 0;
	for( std::string line; getline( file, line ); )
	{
		std::vector <string> fields;
		boost::split(fields,line,boost::is_any_of("\t "),boost::token_compress_on);
		std::string value = "";
		if(fields.size() != 3)
		{
			continue;
		}
		if(std::string::npos == fields.at(2).find("kB"))
		{
			continue;
		}
		if((std::string::npos != fields.at(0).find("MemFree:")) and (std::string::npos == fields.at(0).find("NvMapMemFree:")))
		{
			memfree = std::atoi(fields.at(1).c_str());
			found_entry_count++;
		}
		if(std::string::npos != fields.at(0).find("MemAvailable:"))
		{
			memavail = std::atoi(fields.at(1).c_str());
			found_entry_count++;
		}
		if(std::string::npos != fields.at(0).find("Buffers:"))
		{
			membuffer = std::atoi(fields.at(1).c_str());
			found_entry_count++;
		}
		if((std::string::npos != fields.at(0).find("Cached:")) and (std::string::npos == fields.at(0).find("SwapCached:")))
		{
			memcached = std::atoi(fields.at(1).c_str());
			found_entry_count++;
		}
	}
	file.close();
	if(found_entry_count != 4)
	{
		diagnostic.Level = ERROR;
		diagnostic.Diagnostic_Message = DROPPING_PACKETS;
		diagnostic.Description = "Unable to read all entries for /proc/meminfo.";
		return -1;
	}
	if(Device_Architecture == "x86_64")
	{
		RAMFree_kB = memavail;
	}
	else if(Device_Architecture == "armv7l")
	{
		RAMFree_kB = memavail;
	}
    else if(Device_Architecture == "arm_64")
    {
        RAMFree_kB = memfree + membuffer + memcached;
    }
	else if(Device_Architecture == "aarch64")
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
eros::resource ResourceMonitor::get_resourceused()
{
	eros::resource newresource;
	newresource.Node_Name = Task_Name;
	newresource.PID = PID;
	newresource.CPU_Perc = CPUUsed_perc;
	newresource.RAM_MB = (double)(RAMUsed_kB/1000.0);
	newresource.EnableResourceDiagnostic = resourceusage_diagnostic_enabled;
	return newresource;
}
int ResourceMonitor::get_DISKFree_perc()
{
	char tempstr[128];
	sprintf(tempstr,"df -h");
	std::string res = exec(tempstr,true);
	std::vector <string> lines;
	boost::split(lines,res,boost::is_any_of("\n"),boost::token_compress_on);
	for(std::size_t i = 0; i < lines.size(); ++i)
	{
		std::vector <string> fields;
		boost::split(fields,lines.at(i),boost::is_any_of(" "),boost::token_compress_on);
		std::string mount_point = fields.at(fields.size()-1);
		if(mount_point == "/")
		{
			try
			{	
				std::string tempstr1 = fields.at(fields.size()-2);
				std::size_t length = tempstr1.size();
				std::string disk_used = tempstr1.substr(0,length-1);
				int disk_free = 100 - std::atoi(disk_used.c_str());
				return disk_free;
			}
			catch(std::exception e)
			{
				diagnostic.Level = ERROR;
				diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
				diagnostic.Description = "Cannot parse Disk Usage with Command: " + std::string(tempstr) + " and result: " + res + " and exception: " + e.what();
			}
		}
	}
	diagnostic.Level = ERROR;
	diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
	diagnostic.Description = "Cannot find Disk Usage with Command: " + std::string(tempstr) + " and result: " + res;

	return -1;
}
eros::diagnostic ResourceMonitor::update()
{
	if(diagnostic.Level >= WARN)
	{
		return diagnostic;
	}
	if(PID == -1)
	{
		int id = -1;
		std::string local_node_name;
		local_node_name = Task_Name.substr(0,Task_Name.size());
		boost::replace_all(local_node_name,"/","");
		char tempstr1[512];
		sprintf(tempstr1,"ps aux | grep \"%s\" 2>&1",local_node_name.c_str());
		std::string res = exec(tempstr1,true);
		std::vector <string> lines;
		boost::split(lines,res,boost::is_any_of("\n"),boost::token_compress_on);
		bool found_process = false;
		for(std::size_t i = 0; i < lines.size(); ++i)
		{
			
			if((std::string::npos == lines.at(i).find("sh -c")) and
			   (std::string::npos == lines.at(i).find("grep")) and
			   (std::string::npos == lines.at(i).find("python")))
			{
				
				std::vector <string> fields;
				boost::split(fields,lines.at(i),boost::is_any_of("\t "),boost::token_compress_on);
				if(fields.size() >= 2)
				{
					id =  atoi(fields.at(1).c_str());
					found_process = true;
				}

			}
			if(found_process == true)
			{
				break;
			}
		}
		if((found_process == false) or (id == 0))
		{
			diagnostic.Level = ERROR;
			diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
			diagnostic.Description = "Cannot find PID For Process: " + Task_Name + " by executing: " + std::string(tempstr1);
			return diagnostic;
		}
		PID = id;
	}
	char tempstr[512];
	sprintf(tempstr,"ps -p %d -o %%cpu,vsz 2>&1",PID);
	std::string res = exec(tempstr,true);
	std::vector<std::string> fields;
	boost::split(fields,res,boost::is_any_of(" \n"),boost::token_compress_on);
	if((int)fields.size() != 5 )
	{
		diagnostic.Level = ERROR;
		diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
		diagnostic.Description = "Unable to process command: " + std::string(tempstr) + " with result: " + res;
		return diagnostic;
	}
	try
	{
		CPUUsed_perc = (int)((atof(fields.at(2).c_str()))/(double)processor_count);
		RAMUsed_kB = atoi(fields.at(3).c_str());
	}
	catch(std::exception e)
	{
		diagnostic.Level = ERROR;
		diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
		diagnostic.Description = "Unable to process command: " + std::string(tempstr) + " with result: " + res + " and generated exception: " + e.what();
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
	if(shortterm_buffer_index > SHORTTERM_BUFFER_SIZE)
	{
		shortterm_buffer_index = 0;
		longterm_buffer_RamUsed_kB.push_back(mean);
		if(longterm_buffer_RamUsed_kB.size() > LONGTERM_BUFFER_SIZE)
		{
			longterm_buffer_RamUsed_kB.erase(longterm_buffer_RamUsed_kB.begin());
		}
	}
	if(memoryleak_detectection_enabled == true)
	{
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
				if((d_ramused_kb > 10) && (d_ramused_kb <= 50))
				{
					diagnostic.Level = WARN;
					diagnostic.Diagnostic_Message = RESOURCE_LEAK;
					char tempstr[512];
					sprintf(tempstr,"Found RAM Leak: %f kB/s",(double)((double)d_ramused_kb/((double)SHORTTERM_BUFFER_SIZE*(double)longterm_buffer_RamUsed_kB.size())));
					diagnostic.Description = tempstr;
					return diagnostic;
				}
				else if(d_ramused_kb > 50)
				{
					diagnostic.Level = ERROR;
					diagnostic.Diagnostic_Message = RESOURCE_LEAK;
					char tempstr[512];
					sprintf(tempstr,"Found RAM Leak: %f kiloBytes per second",(double)((double)d_ramused_kb/((double)SHORTTERM_BUFFER_SIZE*(double)longterm_buffer_RamUsed_kB.size())));
					diagnostic.Description = tempstr;
					return diagnostic;
				}

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
std::string ResourceMonitor::exec(const char *cmd, bool wait_for_result)
{
	char buffer[512];
	std::string result = "";
	try
	{
		FILE *pipe = popen(cmd, "r");
	
		if (wait_for_result == false)
		{
			pclose(pipe);
			return "";
		}
		if (pipe == NULL)
		{
			pclose(pipe);
			printf("%s Node: %s popen() failed with command: %s at line: %d\n",__FILE__,Task_Name.c_str(),cmd,__LINE__);
			return "";
		}
		try
		{
			while (!feof(pipe))
			{
				if (fgets(buffer, 512, pipe) != NULL)
				{
					result += buffer;
				}
			}
		}
		catch (...)
		{
			pclose(pipe);
			printf("%s Node: %s popen() failed with command: %s at line: %d\n",__FILE__,Task_Name.c_str(),cmd,__LINE__);
			return "";
		}
		pclose(pipe);
		return result;
	}
	catch(std::exception e)
	{
		printf("%s Node: %s open() failed with command: %s and error: %s at line: %d\n",__FILE__,
			Task_Name.c_str(),
			cmd,
			e.what(),
			__LINE__);
		return "";
	}
}