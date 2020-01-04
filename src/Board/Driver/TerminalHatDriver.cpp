#include "TerminalHatDriver.h"
TerminalHatDriver::TerminalHatDriver()
{
    supported_partnumbers.push_back("100003");
	address = 0;
}
TerminalHatDriver::~TerminalHatDriver()
{
}
bool TerminalHatDriver::init(std::string partnumber_)
{
    bool found_me = false;
    for(std::size_t i = 0; i < supported_partnumbers.size(); ++i)
    {
        if(supported_partnumbers.at(i) == partnumber_)
        {
            found_me = true;
        }
    }
    if(found_me == false)
    {
        printf("[TerminalHatDriver] PartNumber: %s Not Supported. Exiting.",partnumber_.c_str());
        return false;
    }
    partnumber = partnumber_;
    init_pinmap();
    wiringPiSetup();
    return true;
}
bool TerminalHatDriver::configure_pin(std::string pinname,std::string mode)
{
    int pin = -1;
    if(PinMap.find(pinname) != PinMap.end())
    {
        pin = PinMap[pinname];
    }
    else
    {
        printf("Unable to configure Pin: %s\n",pinname.c_str());
        return false;
    }
    if((mode == "DigitalOutput") || (mode == "DigitalOutput-NonActuator"))
    {
        pinMode (pin, OUTPUT);
        return true;
    }
    else if((mode == "DigitalInput") || (mode == "DigitalInput-Safety"))
    {
        pinMode(pin,INPUT);
        return true;
    }
    return false;
}
bool TerminalHatDriver::set_pin(std::string pinname,int v)
{
    int pin = -1;
    if(PinMap.find(pinname) != PinMap.end())
    {
        pin = PinMap[pinname];
    }
    else
    {
        printf("Unable to Set Pin: %s\n",pinname.c_str());
        return false;
    }
    if(v == 1)
    {
        digitalWrite(pin,HIGH);
    }
    else
    {
        digitalWrite(pin,LOW);
    } 
    return true;
}
int TerminalHatDriver::read_pin(std::string pinname)
{
    int pin = -1;
    if(PinMap.find(pinname) != PinMap.end())
    {
        pin = PinMap[pinname];
    }
    else
    {
        printf("Unable to Read Pin: %s\n",pinname.c_str());
        return -1;
    }
    return digitalRead(pin);
}

void TerminalHatDriver::init_pinmap()
{
    //BROADCOM GPIO PIN # TO WIRING PI PIN #
    //Run gpio readall for pin map
    if(partnumber == "100003")
    {
        PinMap["GPIO21"] = 29;
        PinMap["GPIO23"] = 4;
        PinMap["GPIO17"] = 0;
        PinMap["GPIO27"] = 2;
        PinMap["GPIO22"] = 3;
        PinMap["GPIO02"] = 8;
        PinMap["GPIO03"] = 9;
        PinMap["GPIO14"] = 15;
        PinMap["GPIO15"] = 16;
        PinMap["GPIO18"] = 1;
        PinMap["GPIO27"] = 2;
        PinMap["GPIO22"] = 3;
        PinMap["GPIO23"] = 4;
        PinMap["GPIO24"] = 5;
        PinMap["GPIO10"] = 12;
        PinMap["GPIO09"] = 13;
        PinMap["GPIO25"] = 6;
        PinMap["GPIO11"] = 14;
        PinMap["GPIO08"] = 10;
        PinMap["GPIO07"] = 11;
        PinMap["GPIO05"] = 21;
        PinMap["GPIO06"] = 22;
        PinMap["GPIO12"] = 26;
        PinMap["GPIO13"] = 23;
        PinMap["GPIO19"] = 24;
        PinMap["GPIO16"] = 27;
        PinMap["GPIO26"] = 25;
        PinMap["GPIO20"] = 28;
        PinMap["GPIO21"] = 29;       
    }
}
void TerminalHatDriver::print_pinmap()
{
    printf("TerminalHat PinMap for PartNumber: %s:\n",partnumber.c_str());
    std::map<std::string,int>::iterator it = PinMap.begin();
    uint32_t i = 0;
	while (it != PinMap.end())
	{
        printf("[%d] Pin: %s\n",it->first.c_str());
        it++;
	}
}