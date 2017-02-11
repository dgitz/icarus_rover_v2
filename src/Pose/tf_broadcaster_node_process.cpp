#include "tf_broadcaster_node_process.h"

TfBroadcasterNodeProcess::TfBroadcasterNodeProcess()
{
	all_device_info_received = false;
	run_time = 0.0;
}
TfBroadcasterNodeProcess::~TfBroadcasterNodeProcess()
{

}
icarus_rover_v2::diagnostic TfBroadcasterNodeProcess::init(icarus_rover_v2::diagnostic indiag,
		Logger *log,std::string hostname)
{
	myhostname = hostname;
	diagnostic = indiag;
	mylogger = log;
	mydevice.DeviceName = hostname;
    bool file_loaded = false;
    TiXmlDocument system_doc("/home/robot/config/SystemFile.xml");
    bool devicefile_loaded = system_doc.LoadFile();
    std::string errorstring;
    if(devicefile_loaded == true)
    {
        std::vector<icarus_rover_v2::diagnostic> diaglist;
        diaglist = parse_systemfile(diagnostic,system_doc);
        bool diagok = true;
        for(int i = 0; i < diaglist.size(); i++)
        {
            if(diaglist.at(i).Level > NOTICE)
            {
                printf("Error: %s\n",diaglist.at(i).Description.c_str());
                diagok = false;
            }
        }
        if(diagok == true) { file_loaded = true; }
    }
    else
    {
        char tempstr[512];
        sprintf(tempstr,"Unable to load SystemFile.xml");
        mylogger->log_error(tempstr);
        printf("%s\n",tempstr);
        diagnostic.Diagnostic_Type = POSE;
        diagnostic.Diagnostic_Message = INITIALIZING_ERROR;
        diagnostic.Level = ERROR;
        diagnostic.Description = std::string(tempstr);
        return diagnostic;
    }
    if(file_loaded == false)
    {
        mylogger->log_error(diagnostic.Description);
        diagnostic.Diagnostic_Type = POSE;
        diagnostic.Diagnostic_Message = INITIALIZING_ERROR;
        diagnostic.Level = ERROR;
        diagnostic.Description = "Unable to parse SystemFile.xml";   
        return diagnostic;
    }
    else
    {
        char tempstr[512];
        sprintf(tempstr,"Process Initialized");
        mylogger->log_notice(tempstr);
        diagnostic.Diagnostic_Type = POSE;
        diagnostic.Diagnostic_Message = NOERROR;
        diagnostic.Level = NOTICE;
        diagnostic.Description = std::string(tempstr);   
        return diagnostic;
    }
}
icarus_rover_v2::diagnostic TfBroadcasterNodeProcess::update(double dt)
{

    run_time += dt;
   
    diagnostic.Diagnostic_Type = POSE;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = NOERROR;
	diagnostic.Description = "Node Executing.";
	return diagnostic;
}
icarus_rover_v2::diagnostic TfBroadcasterNodeProcess::new_pinmsg(icarus_rover_v2::pin pinmsg)
{
	return diagnostic;
}
icarus_rover_v2::diagnostic TfBroadcasterNodeProcess::new_commandmsg(icarus_rover_v2::command msg)
{
	return diagnostic;
}

icarus_rover_v2::diagnostic TfBroadcasterNodeProcess::new_devicemsg(icarus_rover_v2::device newdevice)
{
	if((newdevice.DeviceName == myhostname) && (all_device_info_received == false))
	{
		mydevice = newdevice;
		all_device_info_received = true;
	}
	return diagnostic;
}
double TfBroadcasterNodeProcess::time_diff(struct timeval timea, struct timeval timeb)
{
	long mtime, seconds, useconds;
	seconds  = timeb.tv_sec  - timea.tv_sec;
	useconds = timeb.tv_usec - timea.tv_usec;

	mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
	return (double)(mtime)/1000.0;
}
std::vector<icarus_rover_v2::diagnostic> TfBroadcasterNodeProcess::parse_systemfile(icarus_rover_v2::diagnostic indiag,TiXmlDocument doc)
{
    icarus_rover_v2::diagnostic diag = indiag;
    std::vector<icarus_rover_v2::diagnostic> diaglist;
	TiXmlElement *l_pRootElement = doc.RootElement();
    bool noerror = true;

	if( NULL != l_pRootElement )
	{
        TiXmlElement *l_pLeverArm = l_pRootElement->FirstChildElement( "LeverArm" );
        while(NULL != l_pLeverArm)
        {
            bool leverarmok = true;
            LeverArm leverarm;
            TiXmlElement *l_pName = l_pLeverArm->FirstChildElement( "Name" );
            if ( NULL != l_pName )
            {
                leverarm.name = l_pName->GetText();
            }
            else
            {
                leverarmok = false;
                diag.Level = ERROR;
                diag.Description = "Could not load Tag: Name";
                diaglist.push_back(diag);
            }
            TiXmlElement *l_pReference = l_pLeverArm->FirstChildElement( "ReferenceLeverArm" );
            if ( NULL != l_pReference )
            {
                leverarm.reference = l_pReference->GetText();
            }
            else
            {
                leverarmok = false;
                diag.Level = ERROR;
                diag.Description = "Could not load Tag: ReferenceLeverArm";
                diaglist.push_back(diag);
            }
            TiXmlElement *l_px = l_pLeverArm->FirstChildElement( "x" );
            if ( NULL != l_px )
            {
                leverarm.x_m = atof(l_px->GetText());
            }
            else
            {
                noerror = false;
                diag.Level = ERROR;
                diag.Description = "Could not load Tag: x";
                diaglist.push_back(diag);
            }
            
            TiXmlElement *l_py = l_pLeverArm->FirstChildElement( "y" );
            if ( NULL != l_py )
            {
                leverarm.y_m = atof(l_py->GetText());
            }
            else
            {
                leverarmok = false;
                diag.Level = ERROR;
                diag.Description = "Could not load Tag:y";
                diaglist.push_back(diag);
            }
            
            TiXmlElement *l_pz = l_pLeverArm->FirstChildElement( "z" );
            if ( NULL != l_pz )
            {
                leverarm.z_m = atof(l_pz->GetText());
            }
            else
            {
                leverarmok = false;
                diag.Level = ERROR;
                diag.Description = "Could not load Tag: z";
                diaglist.push_back(diag);
            }
            
            TiXmlElement *l_proll = l_pLeverArm->FirstChildElement( "roll" );
            if ( NULL != l_proll )
            {
                leverarm.roll_deg = atof(l_proll->GetText());
            }
            else
            {
                leverarmok = false;
                diag.Level = ERROR;
                diag.Description = "Could not load Tag: roll";
                diaglist.push_back(diag);
            }
            
            TiXmlElement *l_ppitch = l_pLeverArm->FirstChildElement( "pitch" );
            if ( NULL != l_px )
            {
                leverarm.pitch_deg = atof(l_ppitch->GetText());
            }
            else
            {
                leverarmok = false;
                diag.Level = ERROR;
                diag.Description = "Could not load Tag: pitch";
                diaglist.push_back(diag);
            }
            
            TiXmlElement *l_pyaw = l_pLeverArm->FirstChildElement( "yaw" );
            if ( NULL != l_pyaw )
            {
                leverarm.yaw_deg = atof(l_pyaw->GetText());
            }
            else
            {
                leverarmok = false;
                diag.Level = ERROR;
                diag.Description = "Could not load Tag: yaw";
                diaglist.push_back(diag);
            }
            if(leverarmok == true)
            {
                noerror = noerror and true;
                LeverArms.push_back(leverarm);
            }
            
            
            
            l_pLeverArm = l_pRootElement->NextSiblingElement("LeverArm");
        }
        
	}
    if(noerror == true)
    {
        diag.Diagnostic_Type = POSE;
        diag.Level = INFO;
        diag.Diagnostic_Message = NOERROR;
        diag.Description = "System File Loaded.";
        diaglist.push_back(diag);
        return diaglist;
    }
    else
    {    
        return diaglist;
    }
}
