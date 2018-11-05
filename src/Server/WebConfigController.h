/*
 * WebConfigController.h
 *
 *  Created on: Nov 5, 2018
 *      Author: robot
 */

#ifndef SRC_ICARUS_ROVER_V2_SRC_SERVER_WEBCONFIGCONTROLLER_H_
#define SRC_ICARUS_ROVER_V2_SRC_SERVER_WEBCONFIGCONTROLLER_H_
#include <mongoose/Server.h>
#include <mongoose/WebController.h>
using namespace Mongoose;
using namespace std;
class WebConfigController : public WebController {
public:
	void hello(Request &request, StreamResponse &response);
	void setup();
};

#endif /* SRC_ICARUS_ROVER_V2_SRC_SERVER_WEBCONFIGCONTROLLER_H_ */
