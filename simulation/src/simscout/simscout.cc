#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include "../../../include/Definitions.h"
#include "../../../include/udpmessage.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

namespace gazebo
{
class SimScout : public ModelPlugin
{
public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
	// Store the pointer to the model
	this->model = _parent;
	udpmessagehandler = new UDPMessageHandler();

	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			std::bind(&SimScout::OnUpdate, this));
	if(initialize_sendsocket() == false)
	{
		printf("Couldn't initialize send socket.  Exiting.\n");
		return;
	}
	printf("Simulation Initialized.");
	/*
      	if(initialize_recvsocket() == false)
      	{
      		printf("Couldn't initialize recv socket.  Exiting.\n");
      		return;
      	}
	 */
}

// Called by the world update start event
public: void OnUpdate()
{
	// Apply a small linear velocity to the model.
	this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
	std::string send_string = udpmessagehandler->encode_EStopUDP("a",3);
	printf("sending: %s\n",send_string.c_str());
	if(sendto(senddevice_sock, send_string.c_str(), send_string.size(), 0, (struct sockaddr *)&senddevice_addr, sizeof(senddevice_addr))!=send_string.size())
	{
		printf("Mismatch in number of bytes sent\n");
	}
	else
	{
		printf("sent\n");
	}

}

// Pointer to the model
private: physics::ModelPtr model;

// Pointer to the update event connection
private: event::ConnectionPtr updateConnection;
UDPMessageHandler *udpmessagehandler;
struct sockaddr_in senddevice_addr;
int senddevice_sock;
std::string send_multicast_group;
int send_multicast_port;
private: bool initialize_sendsocket()
{
	send_multicast_group = "239.255.43.21";
	send_multicast_port = 55555;
	memset(&senddevice_addr, 0, sizeof(senddevice_addr));
	senddevice_addr.sin_family=AF_INET;
	//Create the socket
	if((senddevice_sock=socket(AF_INET, SOCK_DGRAM, 0))<0)
	{
		printf("Failed to create send socket. Exiting.\n");
		return false;
	}

	if(bind(senddevice_sock,( struct sockaddr *) &senddevice_addr, sizeof(senddevice_addr))<0)
	{
		printf("Failed to bind send socket. Exiting.\n");
		return false;
	}
	inet_pton(AF_INET,send_multicast_group.c_str(),&senddevice_addr.sin_addr.s_addr);
	senddevice_addr.sin_port=htons(send_multicast_port);
	return true;
}
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SimScout)
}
