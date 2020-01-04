#include "TerminalHatDriver.h"
TerminalHatDriver::TerminalHatDriver()
{
    supported_partnumbers.push_back("100003");
    supported_partnumbers.push_back("100008");
    supported_partnumbers.push_back("100009");
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
    if(partnumber == "100008")
    {
        PinMap["GPIO8"] = 8;
        PinMap["GPIO9"] = 9;
        PinMap["GPIO7"] = 7;
        PinMap["GPIO15"] = 15;
        PinMap["GPIO16"] = 16;
        PinMap["GPIO0"] = 0;
        PinMap["GPIO1"] = 1;
        PinMap["GPIO2"] = 2;
        PinMap["GPIO3"] = 3;
        PinMap["GPIO4"] = 4;
        PinMap["GPIO5"] = 5;
        PinMap["GPIO12"] = 12;
        PinMap["GPIO13"] = 13;
        PinMap["GPIO6"] = 6;
        PinMap["GPIO14"] = 14;
        PinMap["GPIO10"] = 10;
        PinMap["GPIO11"] = 11;
        PinMap["GPIO30"] = 31;
        PinMap["GPIO21"] = 21;
        PinMap["GPIO22"] = 22;
        PinMap["GPIO26"] = 26;
        PinMap["GPIO23"] = 23;
        PinMap["GPIO24"] = 24;
        PinMap["GPIO27"] = 27;
        PinMap["GPIO25"] = 25;
        PinMap["GPIO28"] = 28;
        PinMap["GPIO29"] = 29;
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