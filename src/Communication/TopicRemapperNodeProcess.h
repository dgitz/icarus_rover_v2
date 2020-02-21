#include "../../include/Base/BaseNodeProcess.cpp"
//C System Files
//C++ System Files
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
//ROS Base Functionality
//ROS Messages
//Project
#include <tinyxml.h>
/*! \class TopicRemapperNodeProcess TopicRemapperNodeProcess.h "TopicRemapperNodeProcess.h"
 *  \brief This is a TopicRemapperNodeProcess class.  Used for the topicremapper_node node.
 *
 */
class TopicRemapperNodeProcess : public BaseNodeProcess
{
public:
	//Constants
	//Enums
	enum OutputModeEnum
	{
		UNKNOWN = 0,
		DIRECT = 1,
		SWITCH =2,
		ARCADEMIX = 3
	};
	//Structs
	struct DrivePerc
	{
		double left;
		double right;
	};
	struct InputChannel
	{
		std::string type;
		std::string topic;
		std::string name;
		int index;
		double minvalue;
		double maxvalue;
	};
	struct OutputChannel
	{
		std::string type;
		std::string topic;
		std::string parentdevice;
		std::string pinname;
		std::string function;
		double maxvalue;
		double minvalue;
		double neutralvalue;
		double deadband;
		double value;
	};
	struct OutputMode
	{
		OutputModeEnum mode;
		std::string type;
		std::string topic;
		std::string name;
		int index;
		int required_value;
	};
	struct TopicMap
	{
		OutputMode outputmode;
		std::vector<InputChannel> ins;
		std::vector<OutputChannel> outs;
	};
	///Initialization Functions
	/*! \brief NodeProcess specific Initialization
	 *
	 */
	eros::diagnostic finish_initialization();
	void reset()
	{
		
	}
	//Update Functions
	/*! \brief Implementation of the update function
	 *
	 */
	eros::diagnostic update(double t_dt, double t_ros_time);

	//Attribute Functions
	std::vector<TopicMap> get_topicmaps() { return TopicMaps; }
	//Message Functions
	/*! \brief  Process Command Message.  All implementation should use at least the code in this Sample Function.
	 *
	 */
	std::vector<eros::diagnostic> new_commandmsg(const eros::command::ConstPtr &t_msg);
	eros::diagnostic new_devicemsg(const eros::device::ConstPtr &device);
	eros::diagnostic new_joymsg(sensor_msgs::Joy joy, std::string topic);

	//Support Functions
	eros::diagnostic load(std::string topicmapfilepath);
	std::vector<eros::pin> get_outputs_pins();
	std::vector<std_msgs::Float32> get_outputs_float32();
	std::vector<std_msgs::Bool> get_outputs_bool();
	DrivePerc arcade_mix(double throttle_perc,double steer_perc);
	//Printing Functions
	std::string print_topicmaps();

protected:
private:
	/*! \brief Process Specific Implementation
	 *
	 */
	OutputModeEnum map_outputmode_enum(std::string v);
	std::string map_outputmode_string(OutputModeEnum v);
	std::vector<eros::diagnostic> check_programvariables();
	eros::diagnostic parse_topicmapfile(eros::diagnostic diag,TiXmlDocument doc);
	double scale_value(double in_value, double neutral_value, double in_min, double in_max, double out_min, double out_max, double deadband);

	std::vector<TopicMap> TopicMaps;

};
