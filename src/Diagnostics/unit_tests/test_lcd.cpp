#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include "../Driver/LCDDriver.h"

using namespace std;

static void show_usage(std::string name)
{
	std::cerr << "Usage: Test LCD. Options:\n"
			<< "\t-h,--help\t\tShow this help message\n"
			<< "\t-r,--red\t\tSet Red Backlight (Default=0)\n"
			<< "\t-g,--green\t\tSet Green Backlight (Default=0)\n"
			<< "\t-b,--blue\t\tSet Blue Backlight (Default=0)\n"
			<< "\t-w,--width\t\tSet Screen Width (Default=20)\n"
			<< "\t-h,--height\t\tSet Screen Height (Default=4)\n"
			<< "\t-m,--mode\t\tOperation Mode \n"
			<< "\t\t Possible Modes: text (Default),colorsweep,colorcycle\n"
			<< "\t\t mode==text:\n"
			<< "\t\t -o,--output\t\t Output Text\n"
			<< "\t\t mode==colorsweep:\n"
			<< "\t\t -o,--output\t\t Output Text\n"
			<< "\t\t mode==colorcycle:\n"
			<< "\t\t No extra Options.\n"
			<< std::endl;
}

int main(int argc, char* argv[])
{
	std::string mode = "text";
	std::string output = "";
	int width = 20;
	int height = 4;
	int red = 0;
	int green = 0;
	int blue = 0;
	int increment = 1;
	if (argc < 2) {
		show_usage(argv[0]);
		return 1;
	}
	for (int i = 1; i < argc; ++i)
	{
		std::string arg = argv[i];
		if ((arg == "-h") || (arg == "--help"))
		{
			show_usage(argv[0]);
			return 0;
		}
		else if ((arg == "-m") || (arg == "--mode"))
		{
			if (i + 1 < argc)
			{
				// Make sure we aren't at the end of argv!
				mode = argv[i+1];
				i++;
			}
			else
			{
				// Uh-oh, there was no argument to the destination option.
				std::cerr << "--mode option requires one argument." << std::endl;
				return 1;
			}
		}
		else if ((arg == "-o") || (arg == "--output"))
		{
			if (i + 1 < argc)
			{
				// Make sure we aren't at the end of argv!
				output = argv[i+1];
				i++;
			}
			else
			{
				// Uh-oh, there was no argument to the destination option.
				std::cerr << "--output option requires one argument." << std::endl;
				return 1;
			}
		}
		else if ((arg == "-r") || (arg == "--red"))
		{
			if (i + 1 < argc)
			{
				// Make sure we aren't at the end of argv!
				red = atoi(argv[i+1]);
				i++;
			}
			else
			{
				// Uh-oh, there was no argument to the destination option.
				std::cerr << "--red option requires one argument." << std::endl;
				return 1;
			}
		}
		else if ((arg == "-g") || (arg == "--green"))
		{
			if (i + 1 < argc)
			{
				// Make sure we aren't at the end of argv!
				green = atoi(argv[i+1]);
				i++;
			}
			else
			{
				// Uh-oh, there was no argument to the destination option.
				std::cerr << "--green option requires one argument." << std::endl;
				return 1;
			}
		}
		else if ((arg == "-b") || (arg == "--blue"))
		{
			if (i + 1 < argc)
			{
				// Make sure we aren't at the end of argv!
				blue = atoi(argv[i+1]);
				i++;
			}
			else
			{
				// Uh-oh, there was no argument to the destination option.
				std::cerr << "--blue option requires one argument." << std::endl;
				return 1;
			}
		}
		else if ((arg == "-w") || (arg == "--width"))
		{
			if (i + 1 < argc)
			{
				// Make sure we aren't at the end of argv!
				width = atoi(argv[i+1]);
				i++;
			}
			else
			{
				// Uh-oh, there was no argument to the destination option.
				std::cerr << "--width option requires one argument." << std::endl;
				return 1;
			}
		}
		else if ((arg == "-h") || (arg == "--height"))
		{
			if (i + 1 < argc)
			{
				// Make sure we aren't at the end of argv!
				height = atoi(argv[i+1]);
				i++;
			}
			else
			{
				// Uh-oh, there was no argument to the destination option.
				std::cerr << "--height option requires one argument." << std::endl;
				return 1;
			}
		}


		else
		{
		}
	}
	LCDDriver lcd;
	int status = lcd.init(width,height);
	if(status <= 0)
	{
		return 0;
	}
	if(mode == "text")
	{
		if(lcd.set_backlightred(red) <= 0)
		{
			printf("Did not set Red Backlight.\n");
		}
		if(lcd.set_backlightgreen(green) <= 0)
		{
			printf("Did not set Green Backlight.\n");
		}
		if(lcd.set_backlightblue(blue) <= 0)
		{
			printf("Did not set Blue Backlight.\n");
		}

		int bytes_written = lcd.send(output);
	}
	else if(mode == "colorcycle")
	{
		LCDDriver::Color start = LCDDriver::RED;
		LCDDriver::Color finish = LCDDriver::WHITE;
		for(int i = (int)start;i < (int)finish;++i)
		{
			char tempstr[48];
			sprintf(tempstr,"Color=%s",lcd.map_color_tostring(static_cast<LCDDriver::Color>(i)).c_str());
			printf("%s\n",tempstr);
			int v = lcd.set_color(static_cast<LCDDriver::Color>(i));
			lcd.send(tempstr);
			usleep(10000000);

		}
	}
	else if(mode == "colorsweep")
	{
		for(int i = 0; i < 100; i+=5)
		{
			if(lcd.set_backlightred(i) <= 0)
			{
				printf("Did not set Red Backlight.\n");
			}
			usleep(10000);
			if(lcd.set_backlightgreen(0) <= 0)
			{
				printf("Did not set Green Backlight.\n");
			}
			usleep(10000);
			if(lcd.set_backlightblue(0) <= 0)
			{
				printf("Did not set Blue Backlight.\n");
			}
			usleep(10000);
			int bytes_written = lcd.send(output);
			usleep(500000);
		}
		for(int i = 0; i < 100; i+=5)
		{
			if(lcd.set_backlightred(0) <= 0)
			{
				printf("Did not set Red Backlight.\n");
			}
			usleep(10000);
			if(lcd.set_backlightgreen(i) <= 0)
			{
				printf("Did not set Green Backlight.\n");
			}
			usleep(10000);
			if(lcd.set_backlightblue(0) <= 0)
			{
				printf("Did not set Blue Backlight.\n");
			}
			usleep(10000);
			int bytes_written = lcd.send(output);
			usleep(500000);
		}
		for(int i = 0; i < 100; i+=5)
		{
			if(lcd.set_backlightred(0) <= 0)
			{
				printf("Did not set Red Backlight.\n");
			}
			usleep(10000);
			if(lcd.set_backlightgreen(0) <= 0)
			{
				printf("Did not set Green Backlight.\n");
			}
			usleep(10000);
			if(lcd.set_backlightblue(i) <= 0)
			{
				printf("Did not set Blue Backlight.\n");
			}
			usleep(10000);
			int bytes_written = lcd.send(output);
			usleep(500000);
		}
	}
	return 0;
}
