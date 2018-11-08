/*
 * WebConfigController.cpp
 *
 *  Created on: Nov 5, 2018
 *      Author: robot
 */

#include "WebConfigController.h"

void WebConfigController::hello(Request &request, StreamResponse &response)
{
	printf("got: %s\n",request.getUrl().c_str());
	response << "{\"msg\":[{\"id\":1}]}" << endl;
}

void WebConfigController::setup()
{
	addRoute("GET", "/request/", WebConfigController, hello);
	addRoute("GET", "/request/device", WebConfigController, hello);
}
