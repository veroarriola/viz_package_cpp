#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <iostream>
#include <vector>

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
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoySubscriber::joy_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Nodo lector de control de PlayStation iniciado. Esperando mensajes en /joy...");
    }

private:
    // Función de callback que se ejecuta al recibir un mensaje
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const
    {
        // Imprime una cabecera para mayor claridad
        RCLCPP_INFO(this->get_logger(), "-----------------------------------------");
        RCLCPP_INFO(this->get_logger(), "Estado del Control de PlayStation Recibido:");

        // Imprime el estado de los botones
        // msg->buttons es un vector de enteros (0 para no presionado, 1 para presionado)
        RCLCPP_INFO(this->get_logger(), "Botones:");
        for (size_t i = 0; i < msg->buttons.size(); ++i)
        {
            // Usamos un stream para construir el mensaje de log
            std::stringstream ss;
            ss << "  Boton " << i << ": " << (msg->buttons[i] ? "Presionado" : "Suelto");
            RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        }

        // Imprime el estado de los ejes
        // msg->axes es un vector de floats, generalmente entre -1.0 y 1.0
        RCLCPP_INFO(this->get_logger(), "Ejes:");
        for (size_t i = 0; i < msg->axes.size(); ++i)
        {
            std::stringstream ss;
            ss << "  Eje " << i << ": " << msg->axes[i];
            RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
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