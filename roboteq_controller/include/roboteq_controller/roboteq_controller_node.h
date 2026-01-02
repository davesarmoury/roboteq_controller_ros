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

#include "roboteq_interfaces/msg/channel_values.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class RoboteqDriver : public rclcpp::Node
{
public:
	explicit RoboteqDriver(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

	~RoboteqDriver(){
		if (ser_.isOpen()){
			ser_.close();
		}
	}

private:
	
	// Serial
	serial::Serial 			ser_;
	std::string 			serial_port_;
	int32_t 				baudrate_;

	// Pub & Sub
	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr 		vel_sub_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr 		serial_read_pub_;
	
	std::vector<rclcpp::Publisher<roboteq_interfaces::msg::ChannelValues>::SharedPtr>  query_pub_;

	rclcpp::TimerBase::SharedPtr 				timer_pub_;

	double 					rpm_scale_;
	std::string 			vel_topic_;

	// queries
	std::map<std::string, std::string>	queries_;

	int frequency_;
	std::mutex 				locker;

	void declare();
	void init();

	void cmdSetup();
	void velCallback(const std_msgs::msg::Float32 &);

	void run();

	void queryCallback();

};