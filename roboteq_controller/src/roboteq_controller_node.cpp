#include "roboteq_controller/roboteq_controller_node.h"

// static const std::string tag {"[RoboteQ] "};
static const std::string tag {""};

namespace roboteq_control
{
hardware_interface::CallbackReturn RoboteqHardware::on_configure(const rclcpp_lifecycle::State & previous_state){
	frequency_ = std::stod(info_.hardware_parameters["frequency"]);
	serial_port_ = info_.hardware_parameters["serial_port"];
	baudrate_ = std::stod(info_.hardware_parameters["baudrate"]);
	gear_ratio_ = std::stof(info_.hardware_parameters["gear_ratio"]);
	max_vel_ = std::stof(info_.hardware_parameters["max_vel"]);

	if (frequency_ <= 0.0){
		RCLCPP_ERROR_STREAM(get_logger(),tag << "Inproper configuration! \'frequency\' need to be greater than zero.");
	}

	queries_["motor_amps"] = "?A";
	queries_["motor_command"] = "?M";
	queries_["fault_flag2"] = "?FF";
	queries_["status_flag"] = "?FS";
	queries_["encoder_speed"] = "?S";

	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoboteqHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::ActuatorInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

	logger_ = std::make_shared<rclcpp::Logger>(
	rclcpp::get_logger("controller_manager.resource_manager.hardware_component.actuator.RoboteqController"));
	clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

	rclcpp::NodeOptions options;
	options.arguments({ "--ros-args", "-r", "__node:=RoboteqControllerInternal"});
	node_ = rclcpp::Node::make_shared("_", options);

	// Initiate communication to serial port
	try{
		ser_.setPort(serial_port_);
		ser_.setBaudrate(baudrate_);
		serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
		ser_.setTimeout(timeout);
		ser_.open();
	}
	catch (serial::IOException &e){
		RCLCPP_ERROR_STREAM(get_logger(),tag << "Unable to open port " << serial_port_);
	    return hardware_interface::CallbackReturn::ERROR;
	}

	if (ser_.isOpen()){
		RCLCPP_INFO_STREAM(get_logger(),tag << "Serial Port " << serial_port_ << " initialized");

		serial_read_pub_ = node_->create_publisher<std_msgs::msg::String>("read", rclcpp::SystemDefaultsQoS());		

		ser_.write("!G 0\r");
		ser_.write("!S 0\r");
		ser_.flush();

		// enable watchdog timer (1000 ms) to stop if no connection
		ser_.write("^RWD 1000\r");

		// closed-loop speed mode
		ser_.write("^MMOD 1\r");
		ser_.flush();

		std::stringstream ss0, ss1;
		ss0 << "^echof 1_";
		ss1 << "# c_/\"DH?\",\"?\"";

		for (auto item : queries_){
			RCLCPP_INFO_STREAM(get_logger(),tag << "Publish topic: " << item.first);
			query_pub_.push_back(node_->create_publisher<std_msgs::msg::String>(item.first, 100));

			std::string cmd = item.second;
			ss1 << cmd << "_";
		}

		ss1 << "# " << frequency_ << "_";
		
		ser_.write(ss0.str());
		ser_.write(ss1.str());
		ser_.flush();

		std::chrono::duration<int, std::milli> dt (1000/frequency_);
		timer_pub_ = node_->create_wall_timer(dt, std::bind(&RoboteqHardware::queryCallback, this) );

		return hardware_interface::CallbackReturn::SUCCESS;
	}
	else{
		RCLCPP_INFO_STREAM(get_logger(),tag << "Serial Port " << serial_port_ << " is not open");
	    return hardware_interface::CallbackReturn::ERROR;
	}
}

float_t RoboteqHardware::radps_2_rpm(float radps){
	return radps * 9.5493; 
}

float_t RoboteqHardware::rpm_2_radps(float rpm){
	return rpm / 9.5493; 
}

hardware_interface::return_type RoboteqHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period){
	std::stringstream cmd_str;

	int vel = static_cast<int>(radps_2_rpm(hw_commands_[0]) * gear_ratio_ / max_vel_ * 1000); 

	cmd_str << "!G " << vel << "_"; // Not sure why closed-loop speed doesn't work

	ser_.write(cmd_str.str());
	ser_.flush();

	return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoboteqHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period){
	std::lock_guard<std::mutex> lock(locker);
	hw_states_[0] = last_vel_;

	return hardware_interface::return_type::OK;
}

void RoboteqHardware::queryCallback(){
	if (ser_.available()){
		std_msgs::msg::String result;

		result.data = ser_.read(ser_.available());
		serial_read_pub_->publish(result);
		
		boost::replace_all(result.data, "\r", "");
		boost::replace_all(result.data, "+", "");

		if(result.data.length() > 2){
			std::vector<std::string> fields;
			
			boost::split(fields, result.data, boost::algorithm::is_any_of("="));
			if (fields.size() < 2){

				RCLCPP_ERROR_STREAM(get_logger(),tag << "Empty data:{" << result.data << "}");
			}

			else if (fields.size() == 2){
				std_msgs::msg::String q_msg;

				q_msg.data = fields[1];
				int i = 0;
				for (auto item : queries_){
					if(item.second == "?" + fields[0]){
						query_pub_[i]->publish(q_msg);
						
						if(fields[0] == "S"){
							std::lock_guard<std::mutex> lock(locker);
							last_vel_ = rpm_2_radps(std::stod(fields[1]));
						}
						break;
					}
					i++;
				}
			}

			else{
				RCLCPP_WARN_STREAM(get_logger(),tag << "Unknown:{" << result.data << "}");
			}
		}
	}
}
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(roboteq_control::RoboteqHardware, hardware_interface::ActuatorInterface)