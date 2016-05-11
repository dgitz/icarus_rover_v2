#include "logger.h"

Logger::Logger()
{
}
Logger::Logger(std::string name)
{
    name.erase(name.begin());
     replace(name.begin(),name.end(),'/','_');
    char buffer[100];
    sprintf(buffer,"%s.out",name.c_str());
    printf("Using name: %s\n",buffer);
    
    sprintf(file_path,"/tmp/output/%s",buffer);
    ofstream log_file;
}
Logger::~Logger()
{
}
void Logger::log_debug(std::string tempstr)
{
    print_log(DEBUG,tempstr);
}
void Logger::log_info(std::string tempstr)
{
    print_log(INFO,tempstr);
}
void Logger::log_notice(std::string tempstr)
{
    print_log(NOTICE,tempstr);
}
void Logger::log_warn(std::string tempstr)
{
    print_log(WARN,tempstr);
}
void Logger::log_error(std::string tempstr)
{
    print_log(ERROR,tempstr);
}
void Logger::log_fatal(std::string tempstr)
{
    print_log(FATAL,tempstr);
}
void Logger::print_log(int level,std::string tempstr)
{
    time_t rawtime;
    struct tm * timeinfo;
    char datebuffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(datebuffer,80,"%d/%m/%Y %I:%M:%S",timeinfo);
    std::string str(datebuffer);
    log_file.open(file_path,ios::out | ios::app | ios::binary|std::ios::ate);
    if(log_file.is_open())
    {
        switch (level)
        {
            case DEBUG:
                log_file << datebuffer << "\tDEBUG\t" << tempstr << endl; 
                break;
            case INFO:
                log_file << datebuffer << "\tINFO\t" << tempstr << endl; 
                break;
            case NOTICE:
                log_file << datebuffer << "\tNOTICE\t" << tempstr << endl; 
                break;
            case WARN:
                log_file << datebuffer << "\tWARN\t" << tempstr << endl;
                break;
            case ERROR:
                log_file << datebuffer << "\tERROR\t" << tempstr << endl; 
                break;
            case FATAL:
                log_file << datebuffer << "\tFATAL\t" << tempstr << endl;
                break;
            default:
                break;
        }   
    }
    log_file.close();
}
