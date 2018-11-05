/*
 * WebConfigController.cpp
 *
 *  Created on: Nov 5, 2018
 *      Author: robot
 */

#include "WebConfigController.h"

void WebConfigController::hello(Request &request, StreamResponse &response)
{
	response << "Hello " << htmlEntities(request.get("name", "... what's your name ?")) << endl;
}

void WebConfigController::setup()
{
	addRoute("GET", "/hello", WebConfigController, hello);
}
