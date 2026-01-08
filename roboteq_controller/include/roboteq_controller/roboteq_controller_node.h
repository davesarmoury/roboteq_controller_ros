#pragma once

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <iostream>
#include <sstream>
#include <typeinfo>
#include <cassert>
#include <mutex>
#include <math.h>
#include <chrono>
#include <memory>
#include <vector>
#include <map>

#include <serial/serial.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float32.hpp"

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using namespace std::chrono_literals;

namespace roboteq_control
{
class RoboteqHardware : public hardware_interface::ActuatorInterface
{
public:
	~RoboteqHardware(){
		if (ser_.isOpen()){
			ser_.close();
		}
	}

	RCLCPP_SHARED_PTR_DEFINITIONS(RoboteqHardware)

	hardware_interface::CallbackReturn on_init(
		const hardware_interface::HardwareInfo & info) override;

	hardware_interface::CallbackReturn on_configure(
		const rclcpp_lifecycle::State & previous_state) override;

	std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

	std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

	hardware_interface::CallbackReturn on_activate(
		const rclcpp_lifecycle::State & previous_state) override;

	hardware_interface::CallbackReturn on_deactivate(
		const rclcpp_lifecycle::State & previous_state) override;

	hardware_interface::return_type read(
		const rclcpp::Time & time, const rclcpp::Duration & period) override;

	hardware_interface::return_type write(
		const rclcpp::Time & time, const rclcpp::Duration & period) override;

	rclcpp::Logger get_logger() const { return *logger_; };


private:
	// Serial
	serial::Serial 			ser_;
	std::string 			serial_port_;
	int 					baudrate_;
	float_t					last_vel_ {0.0};

	// Pub & Sub
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr 		serial_read_pub_;
	std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr>  query_pub_;

	rclcpp::TimerBase::SharedPtr 				timer_pub_;

	double 					gear_ratio_;
	double 					max_vel_;
	std::string 			vel_topic_;

	// queries
	std::map<std::string, std::string>	queries_;

	int 					frequency_;
	std::mutex 				locker;

	void declare();
	void init();

	void cmdSetup();

	void run();

	void queryCallback();

	// Objects for logging
	std::shared_ptr<rclcpp::Logger> logger_;
	rclcpp::Clock::SharedPtr clock_;
	rclcpp::Node::SharedPtr node_;

	float_t radps_2_rpm(float radps);
	float_t rpm_2_radps(float rpm);

	std::vector<double> hw_commands_;
	std::vector<double> hw_states_;
};

}  // namespace roboteq_control