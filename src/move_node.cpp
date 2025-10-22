#include <cmath>
#include <numbers>
#include <thread>
#include <chrono>
#include <memory>
#include <cstdio>

#include <time.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "paquito.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;


class MovementPublisher : public rclcpp::Node
{
public:
	MovementPublisher(rclcpp::NodeOptions options=rclcpp::NodeOptions()) : Node("state_publisher", options)
	{
		// Se suscribe a comandos para asignar la velocidad al robot
		_vel_subscription = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&MovementPublisher::set_velocity, this, _1));

		// Publica para que robot_state_publisher reciba la información sobre las articulaciones
		_joint_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

		// Información tf2 para determinar los ejes del sistema de coordenadas en el sistema 'odom'
		_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

		RCLCPP_INFO(this->get_logger(), "Starting movement state publisher");
		_loop_rate = std::make_shared<rclcpp::Rate>(33ms);
		_timer = this->create_wall_timer(33ms,std::bind(&MovementPublisher::publish, this));
	}

	void set_velocity(const geometry_msgs::msg::Twist);

	void publish();
private:
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _vel_subscription;

	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _joint_publisher;
	std::shared_ptr<tf2_ros::TransformBroadcaster> _broadcaster;
	rclcpp::Rate::SharedPtr _loop_rate;
    rclcpp::TimerBase::SharedPtr _timer;

	rclcpp::Time prev_t = this->get_clock()->now();

	/// Variables de estado del robot
	const double WHEEL_R = 0.0375;
	double front_left_wheel = 0.;
	double rear_left_wheel = 0.;
	double front_right_wheel = 0.;
	double rear_right_wheel = 0.;

	double x = 0., y = 0., roll = 0.;

	// Velocidades angulares
	double w_fl = 0.;
	double w_rl = 0.;
	double w_fr = 0.;
	double w_rr = 0.;

	void publish_wheel_transform(WheelPos&, rclcpp::Time&);
};

void MovementPublisher::set_velocity(const geometry_msgs::msg::Twist vel)
{
	double vx = vel.linear.x;
	double vy = vel.linear.y;
	double wz = vel.angular.z;
	double c = WHEEL_X +  WHEEL_Y;
	w_fl = (vx - vy - c * wz) / WHEEL_R;
	w_rl = (vx + vy - c * wz) / WHEEL_R;
	w_fr = (vx + vy + c * wz) / WHEEL_R;
	w_rr = (vx - vy + c * wz) / WHEEL_R;
}

void MovementPublisher::publish_wheel_transform(WheelPos& wheel_pos, rclcpp::Time& time_stamp)
{
	geometry_msgs::msg::TransformStamped t;

	// Sistemas de coordenadas
	t.header.stamp = time_stamp;

	// odom es el sistema de coordenadas base de tf2
	t.header.frame_id = "chassis";
	// base_link es la base del sistema de coordenadas como fue declarado en el urdf
	t.child_frame_id = wheel_pos.name;


	// Posición de la llanta
	// add translation change
	t.transform.translation.x = wheel_pos.x;
	t.transform.translation.y = wheel_pos.y;
	t.transform.translation.z = -0.03;

	// Ángulo de Euler a cuaternión para indicar la rotación
	tf2::Quaternion q;
	q.setRPY(wheel_pos.roll, 0, 0);
	t.transform.rotation.x = q.x();
	t.transform.rotation.y = q.y();
	t.transform.rotation.z = q.z();
	t.transform.rotation.w = q.w();

	// Publicar
	_broadcaster->sendTransform(t);
}

void MovementPublisher::publish()
{
	
	rclcpp::Time time_stamp = this->get_clock()->now();
	rclcpp::Duration delta_t_duration = time_stamp - prev_t;
	double delta_t = delta_t_duration.seconds();
	//if (delta_t >= 1.)
	//{
		prev_t = time_stamp;
		
		// Mensajes
		geometry_msgs::msg::TransformStamped t;
		sensor_msgs::msg::JointState joint_state;
		

		// Articulaciones
		joint_state.header.stamp = time_stamp;
		// Articulaciones móviles según el urdf
		joint_state.name = {
			"front_left_wheel",
			"rear_left_wheel",
			"front_right_wheel",
			"rear_right_wheel"
		};
		
		joint_state.position = {
			front_left_wheel,
			rear_left_wheel,
			front_right_wheel,
			rear_right_wheel
		};

		// Sistemas de coordenadas
		t.header.stamp = time_stamp;

		// odom es el sistema de coordenadas base de tf2
		t.header.frame_id = "odom";
		// base_link es la base del sistema de coordenadas como fue declarado en el urdf
		t.child_frame_id = "base_link";


		// Posición del robot
		// Cinemática directa
		// https://automaticaddison.com/how-to-simulate-a-mobile-robot-in-gazebo-ros-2-jazzy/#Mecanum_Wheel_Forward_and_Inverse_Kinematics

		// En el sistema de coordenadas del robot
		double v_x = WHEEL_R * (w_fl + w_rl + w_fr + w_rr) / 4;
		double v_y = WHEEL_R * (-w_fl + w_rl + w_fr - w_rr) / 4;
		double w_z = WHEEL_R * (-w_fl - w_rl + w_fr + w_rr) / (4 * (WHEEL_X + WHEEL_Y));
		double delta_x = v_x * delta_t;
		double delta_y = v_y * delta_t;
		x += delta_x * cos(roll) - delta_y * sin(roll);
		y += delta_x * sin(roll) + delta_y * cos(roll);
		roll += w_z * delta_t;

		// Pasar a sistema de coordenadas global
		// add translation change
		t.transform.translation.x = x ;
		t.transform.translation.y = y;
		t.transform.translation.z = 0;

		// Ángulo de Euler a cuaternión para indicar la rotación
		tf2::Quaternion q;
		q.setRPY(0, 0, roll);
		t.transform.rotation.x = q.x();
		t.transform.rotation.y = q.y();
		t.transform.rotation.z = q.z();
		t.transform.rotation.w = q.w();


		// Actualizar el estado
		front_left_wheel += w_fl * delta_t;
		front_right_wheel += w_fr * delta_t;
		rear_right_wheel += w_rr * delta_t;
		rear_left_wheel += w_rl * delta_t;

		// Publicar
		_broadcaster->sendTransform(t);
		publish_wheel_transform(fl, time_stamp);
		publish_wheel_transform(fr, time_stamp);
		publish_wheel_transform(rr, time_stamp);
		publish_wheel_transform(rl, time_stamp);
		_joint_publisher->publish(joint_state);
	//}
}

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MovementPublisher>());
	rclcpp::shutdown();
	return 0;
}

