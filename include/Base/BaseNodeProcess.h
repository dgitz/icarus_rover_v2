// Base class
using namespace std;
class BaseNodeProcess {
public:
	virtual ~BaseNodeProcess()
	{
	}
	void init(std::string _base_node_name,std::string _node_name)
	{
		run_time = 0.0;
		base_node_name = _base_node_name;
		node_name = _node_name;
	}
	bool update(double dt);
	void print_runtime();
	void set_diagnostic(icarus_rover_v2::diagnostic _diagnostic) { diagnostic = _diagnostic; }
	icarus_rover_v2::diagnostic get_diagnostic() { return diagnostic; }




protected:
private:
	double run_time;
	icarus_rover_v2::diagnostic diagnostic;
	std::string base_node_name,node_name;
};
