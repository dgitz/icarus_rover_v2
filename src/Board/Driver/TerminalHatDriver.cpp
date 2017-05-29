#include "TerminalHatDriver.h"
TerminalHatDriver::TerminalHatDriver()
{
	address = 0;
}
TerminalHatDriver::~TerminalHatDriver()
{
}
void TerminalHatDriver::init()
{
}
bool TerminalHatDriver::configure_pin(int pinnumber,std::string mode)
{
    int v = map_connectorpin_to_pinfile(pinnumber);
    if(v == -1) { return false; }
    std::string export_str = "/sys/class/gpio/export";
    std::ofstream exportgpio(export_str.c_str()); // Open "export" file. Convert C++ string to C string. Required for all Linux pathnames
    if (exportgpio < 0)
    {
        printf("OPERATION FAILED: Unable to export GPIO %d/%d\n",pinnumber,map_connectorpin_to_pinfile(pinnumber));
        return false;
    }

    exportgpio << map_connectorpin_to_pinfile(pinnumber); //write GPIO number to export
    exportgpio.close(); //close export file
    if(mode == "DigitalInput")
    {
        //std::string setdir_str ="/sys/class/gpio/gpio" + std::to_string(map_connectorpin_to_pinfile(pinnumber)) + "/direction";
        std::ostringstream setdir_str;
        setdir_str << "/sys/class/gpio/gpio" << map_connectorpin_to_pinfile(pinnumber) << "/direction";    
        std::ofstream setdirgpio(setdir_str.str().c_str()); // open direction file for gpio
        if (setdirgpio < 0)
        {
            printf(" OPERATION FAILED: Unable to set direction of GPIO %d/%d\n",pinnumber,map_connectorpin_to_pinfile(pinnumber));
            return false;
        }
        setdirgpio << "in"; //write direction to direction file
        setdirgpio.close(); // close direction file
        return true;
    }
    else if((mode == "DigitalOutput") || (mode == "DigitalOutput-NonActuator"))
    {
        //std::string setdir_str ="/sys/class/gpio/gpio" + std::to_string(map_connectorpin_to_pinfile(pinnumber)) + "/direction";
        std::ostringstream setdir_str;
        setdir_str << "/sys/class/gpio/gpio" << map_connectorpin_to_pinfile(pinnumber) << "/direction";    
        std::ofstream setdirgpio(setdir_str.str().c_str()); // open direction file for gpio
        if (setdirgpio < 0)
        {
            printf(" OPERATION FAILED: Unable to set direction of GPIO %d/%d\n",pinnumber,map_connectorpin_to_pinfile(pinnumber));
            return false;
        }
        setdirgpio << "out"; //write direction to direction file
        setdirgpio.close(); // close direction file
        return true;
    }
    
    else
    {
        return false;
    }
}
bool TerminalHatDriver::set_pin(int pinnumber, int v)
{
    std::ostringstream setval_str;
    setval_str << "/sys/class/gpio/gpio" <<  map_connectorpin_to_pinfile(pinnumber) <<  "/value";
    std::ofstream setvalgpio(setval_str.str().c_str()); // open value file for gpio
    if (setvalgpio < 0)
    {
        printf("OPERATION FAILED: Unable to set the value of GPIO: %d/%d\n",pinnumber,map_connectorpin_to_pinfile(pinnumber));
        return false;
    }
    setvalgpio << v ;//write value to value file
    setvalgpio.close();// close value file
    return true;
}
int TerminalHatDriver::read_pin(int pinnumber)
{
    std::ostringstream getval_str;
    getval_str << "/sys/class/gpio/gpio" << map_connectorpin_to_pinfile(pinnumber) << "/value";
    std::ifstream getvalgpio(getval_str.str().c_str());// open value file for gpio
    if (getvalgpio < 0)
    {
        printf("OPERATION FAILED: Unable to get value of GPIO %d/%d\n",pinnumber,map_connectorpin_to_pinfile(pinnumber));
        return -1;
    }
    std::string val;
    getvalgpio >> val ;  //read gpio value
    int readvalue = 0;
    if(val != "0")
    {
        readvalue = 1;
    }

    getvalgpio.close(); //close the value file
    return readvalue;
}
/*
int TerminalHatDriver::map_pinfile_to_connectorpin(std::string)
{
    return -1;
}
*/
int TerminalHatDriver::map_connectorpin_to_pinfile(int pinnumber)
{
    switch(pinnumber)
    {
        case 3:     return 2;   break;
        case 5:     return 3;   break;
        case 7:     return 4;   break;
        case 11:    return 17;  break;
        case 13:    return 27;  break;
        case 15:    return 22;  break;
        case 19:    return 10;  break;
        case 21:    return 9;   break;
        case 23:    return 11;  break;
        case 27:    return 0;   break;
        case 29:    return 5;   break;
        case 31:    return 6;   break;
        case 33:    return 13;  break;
        case 35:    return 19;  break;
        case 37:    return 26;  break;
        case 8:     return 14;  break;
        case 10:    return 15;  break;
        case 12:    return 18;  break;
        case 16:    return 23;  break;
        case 18:    return 24;  break;
        case 22:    return 25;  break;
        case 24:    return 8;   break;
        case 26:    return 7;   break;
        case 28:    return 1;   break;
        case 32:    return 12;  break;
        case 36:    return 16;  break;
        case 38:    return 20;  break;
        case 40:    return 21;  break;
        default:    return -1;  break;
    }
}