#include "../include/Base/BaseNodeProcess.cpp"
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
class TopicRemapperNodeProcess: public BaseNodeProcess {
public:
    //Constants
    //Enums
    //Structs
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
	        int pinnumber;
	        std::string function;
	        double maxvalue;
	        double minvalue;
	        double neutralvalue;
	        double deadband;
	        double value;
	    };
	    struct OutputMode
	    {
	        std::string mode;
	        std::string type;
	        std::string topic;
	        std::string name;
	        int index;
	        int required_value;
	    };
	    struct TopicMap
	    {
	        OutputMode outputmode;
	        InputChannel in;
	        std::vector<OutputChannel> outs;
	    };
	///Initialization Functions
	/*! \brief NodeProcess specific Initialization
	 *
	 */
	eros::diagnostic finish_initialization();
	//Update Functions
	/*! \brief Implementation of the update function
	 *
	 */
	eros::diagnostic update(double t_dt,double t_ros_time);

	//Attribute Functions
	std::vector<TopicMap> get_topicmaps() { return TopicMaps; }
	//Message Functions
	/*! \brief  Process Command Message.  All implementation should use at least the code in this Sample Function.
	 *
	 */
	std::vector<eros::diagnostic> new_commandmsg(const eros::command::ConstPtr& t_msg);
	eros::diagnostic new_devicemsg(const eros::device::ConstPtr& device);
	 eros::diagnostic new_joymsg(sensor_msgs::Joy joy,std::string topic);

	//Support Functions
	eros::diagnostic load(std::string topicmapfilepath);
	std::vector<eros::pin> get_outputs_pins();
	std::vector<std_msgs::Float32> get_outputs_float32();
    //Printing Functions
	std::string print_topicmaps();
protected:
private:
	/*! \brief Process Specific Implementation
	 *
	 */
	std::vector<eros::diagnostic> check_programvariables();
	int parse_topicmapfile(TiXmlDocument doc);
	    double scale_value(double in_value,double neutral_value,double in_min,double in_max,double out_min,double out_max, double deadband);


	    std::vector<TopicMap> TopicMaps;
};
