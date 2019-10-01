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
bool TerminalHatDriver::configure_pin(std::string pinname,std::string mode)
{
    int pin_number = map_connectorpin_to_pinfile(pinname);
    if(pin_number == -1)
    {
        printf("[ERROR]: PinName: %s is not available!\n",pinname.c_str());
        return -1;
    }
    return configure_pin(pin_number,mode);
}
bool TerminalHatDriver::configure_pin(int pinnumber,std::string mode)
{
    int v = map_connectorpin_to_pinfile(pinnumber);
    if(v == -1)
    {
    	printf("Could not map pin: %d to connector.\n",pinnumber);
    	return false;
    }
    std::string export_str = "/sys/class/gpio/export";
    std::ofstream exportgpio(export_str.c_str()); // Open "export" file. Convert C++ string to C string. Required for all Linux pathnames
    if (exportgpio < 0)
    {
        printf("OPERATION FAILED: Unable to export GPIO %d/%d\n",pinnumber,map_connectorpin_to_pinfile(pinnumber));
        return false;
    }
    exportgpio << map_connectorpin_to_pinfile(pinnumber); //write GPIO number to export
    exportgpio.close(); //close export file
    if((mode == "DigitalInput") or (mode == "DigitalInput-Safety"))
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
        char tempstr[128];
        sprintf(tempstr,"raspi-gpio set %d pu\n",map_connectorpin_to_pinfile(pinnumber));
        int v = system(tempstr);
        /*
        std::ostringstream pullupdown_str;
        pullupdown_str << "raspi-gpio set " << map_connectorpin_to_pinfile(pinnumber) << "pu";
        std::ofstream setpullupdown(pullupdown_str.str().c_str()); //
        if (setpullupdown < 0)
        {
        	printf(" OPERATION FAILED: Unable to set Pullup/down of GPIO %d/%d\n",pinnumber,map_connectorpin_to_pinfile(pinnumber));
        	return false;
        }
        */
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
    	printf("Unsupported Pin Function: %s\n",mode.c_str());
        return false;
    }
    return false;
}
bool TerminalHatDriver::set_pin(std::string pinname,int v)
{
    int pin_number = map_connectorpin_to_pinfile(pinname);
    if(pin_number == -1)
    {
        printf("[ERROR]: PinName: %s is not available!\n",pinname.c_str());
        return -1;
    }
    return set_pin(pin_number,v);
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
int TerminalHatDriver::read_pin(std::string pinname)
{
    int pin_number = map_connectorpin_to_pinfile(pinname);
    if(pin_number == -1)
    {
        printf("[ERROR]: PinName: %s is not available!\n",pinname.c_str());
        return -1;
    }
    return read_pin(pin_number);
}
int TerminalHatDriver::read_pin(int pinnumber)
{
    std::ostringstream getval_str;
    getval_str << "/sys/class/gpio/gpio" << map_connectorpin_to_pinfile(pinnumber) << "/value";
    std::ifstream getvalgpio(getval_str.str().c_str());// open value file for gpio
    if (getvalgpio < 0)
    {
        printf("[ERROR]: OPERATION FAILED: Unable to get value of GPIO %d/%d\n",pinnumber,map_connectorpin_to_pinfile(pinnumber));
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
int TerminalHatDriver::map_connectorpin_to_pinfile(std::string pinname)
{
    if(pinname == "") { return -1; }
    else if(pinname == "GPIO02") { return -1; } //USED FOR I2C
    else if(pinname == "GPIO03") { return -1; } //USED FOR I2C
    else if(pinname == "GPIO04") { return 4; }
    else if(pinname == "GPIO17") { return 17; }
    else if(pinname == "GPIO27") { return 27; }
    else if(pinname == "GPIO22") { return 22; }
    else if(pinname == "GPIO10") { return -1; } //USED FOR SPI
    else if(pinname == "GPIO09") { return -1; } //USED FOR SPI
    else if(pinname == "GPIO11") { return -1; } //USED FOR SPI
    else if(pinname == "ID_SD") { return -1; } //USED FOR SPI
    else if(pinname == "GPIO05") { return 5; }
    else if(pinname == "GPIO06") { return 6; }
    else if(pinname == "GPIO13") { return 13; }
    else if(pinname == "GPIO19") { return 19; }
    else if(pinname == "GPIO26") { return 26; }
    else if(pinname == "GPIO14") { return -1; } //USED FOR UART
    else if(pinname == "GPIO15") { return -1; } //USED FOR UART
    else if(pinname == "GPIO18") { return 18; }
    else if(pinname == "GPIO23") { return 23; }
    else if(pinname == "GPIO24") { return 24; }
    else if(pinname == "GPIO25") { return 25; }
    else if(pinname == "GPIO08") { return -1; } //USED FOR SPI
    else if(pinname == "GPIO07") { return -1; } //USED FOR SPI
    else if(pinname == "ID_SC") { return -1; } //USED FOR I2C
    else if(pinname == "GPIO12") { return 12; }
    else if(pinname == "GPIO16") { return 16; }
    else if(pinname == "GPIO20") { return 20; }
    else if(pinname == "GPIO21") { return 21; }
    else { return -1; }
    
}
int TerminalHatDriver::map_connectorpin_to_pinfile(int pinnumber)
{
    switch(pinnumber)
    {
        case 3:     return -1;   break; //USED FOR I2C
        case 5:     return -1;   break; //USED FOR I2C
        case 7:     return 4;   break;
        case 11:    return 17;  break;
        case 13:    return 27;  break;
        case 15:    return 22;  break;
        case 19:    return -1;  break; //USED FOR SPI
        case 21:    return -1;   break; //USED FOR SPI
        case 23:    return -1;  break; //USED FOR SPI
        case 27:    return -1;   break; //USED FOR SPI
        case 29:    return 5;   break;
        case 31:    return 6;   break;
        case 33:    return 13;  break;
        case 35:    return 19;  break;
        case 37:    return 26;  break;
        case 8:     return -1;  break; //USED FOR UART
        case 10:    return -1;  break; //USED FOR UART
        case 12:    return 18;  break;
        case 16:    return 23;  break;
        case 18:    return 24;  break;
        case 22:    return 25;  break;
        case 24:    return -1;   break; //USED FOR SPI
        case 26:    return -1;   break; //USED FOR SPI
        case 28:    return -1;   break; //USED FOR I2C
        case 32:    return 12;  break;
        case 36:    return 16;  break;
        case 38:    return 20;  break;
        case 40:    return 21;  break;
        default:    return -1;  break;
    }
}
