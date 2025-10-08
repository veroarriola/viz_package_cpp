#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <iostream>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>

#include "paquito.hpp"

// Generado con ayuda de Gemini 2.5 Pro y adaptado
using std::placeholders::_1;

// La clase hereda de rclcpp::Node, la estructura base de un nodo en ROS 2
class JoySubscriber : public rclcpp::Node
{
public:
    // Constructor
    JoySubscriber() : Node("ps_controller_reader")
    {
        // Creamos la suscripción.
        // El tipo de mensaje es sensor_msgs::msg::Joy.
        // El '10' es la profundidad de la cola (Quality of Service).
        // El callback se enlaza usando std::bind.
        _subscription = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoySubscriber::joy_callback, this, _1));

        // Publicará las velocidades solicitadas según el uso del control PS2
        _vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);


        RCLCPP_INFO(this->get_logger(), "Nodo de control de PlayStation iniciado. Esperando mensajes en /joy...");
    }

private:
    // Función de callback que se ejecuta al recibir un mensaje
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        geometry_msgs::msg::Twist twist;

        if (msg->buttons[0]) {
            speed -= 1;
        } else if(msg->buttons[2]) {
            speed += 1;
        }

        twist.linear.x = msg->axes[0] * speed;
        twist.linear.y = msg->axes[1] * speed;
        twist.angular.z = msg->axes[3] * speed;
        

        // Imprime el estado de los ejes
        // msg->axes es un vector de floats, generalmente entre -1.0 y 1.0
        RCLCPP_INFO(this->get_logger(), "Ejes:");
        for (size_t i = 0; i < msg->axes.size(); ++i)
        {
            std::stringstream ss;
            ss << "  Eje " << i << ": " << msg->axes[i];
            RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        }

        _vel_publisher->publish(twist);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _subscription;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _vel_publisher;
    double speed = 0;
};

int main(int argc, char * argv[])
{
    // Inicializa ROS 2
    rclcpp::init(argc, argv);

    // Crea una instancia del nodo y lo mantiene vivo hasta que se detenga (Ctrl+C)
    rclcpp::spin(std::make_shared<JoySubscriber>());

    // Libera los recursos de ROS 2
    rclcpp::shutdown();
    return 0;
}