#include "roboteq_controller/roboteq_controller_node.h"

// static const std::string tag {"[RoboteQ] "};
static const std::string tag {""};

void RoboteqDriver::declare(){
	declare_parameter<std::string>("serial_port", "dev/ttyUSB0");
	declare_parameter("baudrate", 115200);
	declare_parameter("rpm_scale", 25.0);
	declare_parameter("max_vel", 3500.0);
	declare_parameter<int>("frequency", 0);

	declare_parameter<std::string>("vel_topic", "/vel");
}

void RoboteqDriver::init(){
	RCLCPP_INFO(get_logger(), "Creating");
	get_parameter("frequency", frequency_);

	get_parameter("serial_port", serial_port_);
	get_parameter("baudrate", baudrate_);
	get_parameter("rpm_scale", rpm_scale_);
	get_parameter("max_vel", max_vel_);
	get_parameter("vel_topic", vel_topic_);

	if (frequency_ <= 0.0){
		RCLCPP_ERROR_STREAM(this->get_logger(),tag << "Inproper configuration! \'frequency\' need to be greater than zero.");
	}

	auto param_interface = this->get_node_parameters_interface();
	std::map<std::string, rclcpp::ParameterValue> params = param_interface->get_parameter_overrides();

	RCLCPP_INFO_STREAM(this->get_logger(), tag << "queries:" );

	for (auto iter = params.begin(); iter != params.end(); iter++){
		std::size_t pos = iter->first.find("query");
		if (pos != std::string::npos && 
			iter->second.get_type() == rclcpp::ParameterType::PARAMETER_STRING){
  			std::string topic = iter->first.substr (pos+ 6);    
			auto query = iter->second.to_value_msg().string_value;
			
			queries_[topic] =  query;
			RCLCPP_INFO(this->get_logger(), "%15s : %s",  topic.c_str(), query.c_str() );
		}
	}
}

RoboteqDriver::RoboteqDriver(const rclcpp::NodeOptions &options): Node("roboteq_controller", options),
	rpm_scale_(1.0),
	frequency_(0),
	serial_port_("dev/ttyUSB0"),
	baudrate_(115200){
	
	declare();
	init();

	vel_sub_ = create_subscription<std_msgs::msg::Float32>(
										vel_topic_, rclcpp::SystemDefaultsQoS(),
										std::bind(&RoboteqDriver::velCallback, this, std::placeholders::_1));

	// Initiate communication to serial port
	try{
		ser_.setPort(serial_port_);
		ser_.setBaudrate(baudrate_);
		serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
		ser_.setTimeout(timeout);
		ser_.open();
	}
	catch (serial::IOException &e){
		RCLCPP_ERROR_STREAM(this->get_logger(),tag << "Unable to open port " << serial_port_);
		rclcpp::shutdown();
	}

	if (ser_.isOpen()){
		RCLCPP_INFO_STREAM(this->get_logger(),tag << "Serial Port " << serial_port_ << " initialized");
	}
	else{
		RCLCPP_INFO_STREAM(this->get_logger(),tag << "Serial Port " << serial_port_ << " is not open");
		rclcpp::shutdown();
	}

	cmdSetup();

	run();
}


void RoboteqDriver::cmdSetup(){
	// stop motors
	ser_.write("!G 0\r");
	ser_.write("!S 0\r");
	ser_.flush();

	// // disable echo
	// ser.write("^ECHOF 1\r");
	// ser.flush();

	// enable watchdog timer (1000 ms) to stop if no connection
	ser_.write("^RWD 1000\r");

	// closed-loop speed mode
	ser_.write("^MMOD 1\r");
	ser_.flush();
}


void RoboteqDriver::run(){
	std::stringstream ss0, ss1;
	ss0 << "^echof 1_";
	ss1 << "# c_/\"DH?\",\"?\"";

	for (auto item : queries_){
		RCLCPP_INFO_STREAM(this->get_logger(),tag << "Publish topic: " << item.first);
		query_pub_.push_back(create_publisher<std_msgs::msg::String>(item.first, 100));

		std::string cmd = item.second;
		ss1 << cmd << "_";
	}

	ss1 << "# " << frequency_ << "_";
	
	ser_.write(ss0.str());
	ser_.write(ss1.str());
	ser_.flush();
	

    serial_read_pub_ = create_publisher<std_msgs::msg::String>("read", rclcpp::SystemDefaultsQoS());

	std::chrono::duration<int, std::milli> dt (1000/frequency_);
	timer_pub_ = create_wall_timer(dt, std::bind(&RoboteqDriver::queryCallback, this) );
}

void RoboteqDriver::velCallback(const std_msgs::msg::Float32 &msg){
	std::stringstream cmd_str;
	int vel = static_cast<int>(msg.data * rpm_scale_ / max_vel_ * 1000 * DEG_2_RAD);

	cmd_str << "!G " << vel << "_"; // Not sure why closed-loop speed doesn't work

	ser_.write(cmd_str.str());
	ser_.flush();
	// RCLCPP_INFO_STREAM(this->get_logger(),cmd_str.str());
}

void RoboteqDriver::queryCallback(){
	auto current_time = this->now();
	if (ser_.available()){
		std_msgs::msg::String result;

		std::lock_guard<std::mutex> lock(locker);

		result.data = ser_.read(ser_.available());

		// std::lock_guard<std::mutex> unlock(locker);


		serial_read_pub_->publish(result);
		
		boost::replace_all(result.data, "\r", "");
		boost::replace_all(result.data, "+", "");

		if(result.data.length() > 2){
			std::vector<std::string> fields;
			
			boost::split(fields, result.data, boost::algorithm::is_any_of("="));
			if (fields.size() < 2){

				RCLCPP_ERROR_STREAM(this->get_logger(),tag << "Empty data:{" << result.data << "}");
			}

			else if (fields.size() == 2){
				std_msgs::msg::String q_msg;

				q_msg.data = fields[1];
				int i = 0;
				for (auto item : queries_){
					if(item.second == "?" + fields[0]){
						query_pub_[i]->publish(q_msg);
						break;
					}
					i++;
				}
			}

			else{
				RCLCPP_WARN_STREAM(this->get_logger(),tag << "Unknown:{" << result.data << "}");
			}
		}
	}
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoboteqDriver>());
  rclcpp::shutdown();
  return 0;
}

