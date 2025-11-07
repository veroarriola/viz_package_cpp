#include <cmath>
#include <chrono>
#include <numbers>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "paquito.hpp"

using namespace std::chrono_literals;

class CirclePublisher : public rclcpp::Node
{
public:
	CirclePublisher() : Node("circle_controller")
	{
		_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
		_loop_rate = std::make_shared<rclcpp::Rate>(33ms);
		_timer = this->create_wall_timer(33ms,std::bind(&CirclePublisher::trace_curve, this));
		RCLCPP_INFO(this->get_logger(), "Publicando círculos en /cmd_vel...");
	}


private:
	void trace_curve()
	{
		rclcpp::Time time = this->get_clock()->now();
		//double secs = time.seconds();

		geometry_msgs::msg::Twist twist;

		twist.linear.x = 0;
		twist.linear.y = -D_PI * radio * freq;
		twist.angular.z = D_PI * freq;

		this->_vel_publisher->publish(twist);
	}

	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _vel_publisher;
	rclcpp::Rate::SharedPtr _loop_rate;
	rclcpp::TimerBase::SharedPtr _timer;
	const double D_PI = 2.0 * M_PI;
	double freq = 0.25;  // Vueltas por segundo
	double radio = 1.5;  // Radio del círculo
};


int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CirclePublisher>());
	rclcpp::shutdown();
	return 0;
}

