#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include "gpioclass.h"

using namespace std;

int main (void)
{
	string inputstate;
    GPIOClass* gpio4 = new GPIOClass("4"); //create new GPIO object to be attached to  GPIO4
    GPIOClass* gpio5 = new GPIOClass("5"); //create new GPIO object to be attached to  GPIO5
	gpio4->export_gpio(); //export GPIO4
    gpio5->export_gpio(); //export GPIO5
	cout << " GPIO pins exported" << endl;

    gpio5->setdir_gpio("in"); //GPIO5 set to input
    gpio4->setdir_gpio("out"); // GPIO4 set to output

    cout << " Set GPIO pin directions" << endl;
	 while(1)
    {
        usleep(100000);  // wait for 0.5 seconds
		cout << "Turning On." << endl;
		gpio4->setval_gpio("1"); // turn LED ON
		gpio5->getval_gpio(inputstate);
		cout << " Got: " << inputstate << endl;
		usleep(100000);  // wait for 0.5 seconds
		cout << "Turning off." << endl;
		gpio4->setval_gpio("0"); // turn LED ON
		gpio5->getval_gpio(inputstate);
		cout << " Got: " << inputstate << endl;
	}
	return 0;
}
/*
   

    

    

    while(1)
    {
        usleep(500000);  // wait for 0.5 seconds
        gpio17->getval_gpio(inputstate); //read state of GPIO17 input pin
        cout << "Current input pin state is " << inputstate  <<endl;
        if(inputstate == "0") // if input pin is at state "0" i.e. button pressed
        {
            //cout << "input pin state is "Pressed ".n Will check input pin state again in 20ms "<<endl;
                usleep(20000);
                    cout << "Checking again ....." << endl;
                    gpio17->getval_gpio(inputstate); // checking again to ensure that state "0" is due to button press and not noise
            if(inputstate == "0")
            {
                //cout << "input pin state is definitely "Pressed". Turning LED ON" <<endl;
                gpio4->setval_gpio("1"); // turn LED ON

                cout << " Waiting until pin is unpressed....." << endl;
                while (inputstate == "0"){
                gpio17->getval_gpio(inputstate);
                };
                cout << "pin is unpressed" << endl;

            }
            else
                //cout << "input pin state is definitely "UnPressed". That was just noise." <<endl;

        }
        gpio4->setval_gpio("0");

    }
    cout << "Exiting....." << endl;
    return 0;
}
*/