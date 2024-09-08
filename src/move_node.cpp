#include <chrono>
#include <memory>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class MovementPublisher : public rclcpp::Node
{
public:
	MovementPublisher() : Node("state_publisher")
	{
		_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
	}
private:
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _publisher;
};


int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MovementPublisher>());
	rclcpp::shutdown();
	return 0;
}

