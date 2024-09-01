#include <chrono>
#include <memory>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

class MarkerPublisher : public rclcpp::Node
{
public:
	MarkerPublisher() : Node("marker_publisher"), _count(0)
	{
		_publisher = this->create_publisher<visualization_msgs::msg::Marker>("marker_topic", 10);
		//while(!_publisher->get_subscription_count()) {
		//	sleep(1);
		//	RCLCPP_INFO(this->get_logger(), "Waiting for subscribers...");
		//}
		auto timer_callback =
			[this]() -> void {
				auto marker = visualization_msgs::msg::Marker();
				marker.header.frame_id = "/base_link";
				marker.header.stamp = this->get_clock()->now();

				// set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
				marker.type = 1;
				marker.id = 0; 

				// Set the scale of the marker
				marker.scale.x = 1.0;
				marker.scale.y = 1.0;
				marker.scale.z = 1.0;

				// Set the color
				marker.color.r = 0.0;
				marker.color.g = 1.0;
				marker.color.b = 0.0;
				marker.color.a = 0.5;

				marker.lifetime.sec = 3;

				// Set the pose of the marker
				marker.pose.position.x = 5.0;
				marker.pose.position.y = 0.0;
				marker.pose.position.z = 0.0;
				marker.pose.orientation.x = 0.0;
				marker.pose.orientation.y = 0.0;
				marker.pose.orientation.z = 0.0;
				marker.pose.orientation.w = 1.0;
				
				this->_publisher->publish(marker);
			};
		_timer = this->create_wall_timer(1000ms, timer_callback);
	}
private:
	rclcpp::TimerBase::SharedPtr _timer;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _publisher;
	size_t _count;
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MarkerPublisher>());
	rclcpp::shutdown();
	return 0;
}

