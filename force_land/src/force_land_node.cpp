#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>

using namespace std::chrono_literals;

class ForceLand : public rclcpp::Node
{
public:
    ForceLand() : Node("force_land"), need_land(false), landing_triggered_(false)
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // Subscriber for altitude
        subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position",
            qos,
            std::bind(&ForceLand::height_callback, this, std::placeholders::_1));

        // NEW: Subscriber to detect if the drone has landed
        subscription_land_detected_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>(
            "/fmu/out/vehicle_land_detected",
            qos,
            std::bind(&ForceLand::land_detected_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

        timer_ = this->create_wall_timer(10ms, std::bind(&ForceLand::activate_switch, this));
    }

private:
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_;

    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr subscription_land_detected_;

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    bool need_land;
    bool landing_triggered_; // Flag to track if landing was triggered

    // Callback per controllare se il drone è a terra
    void land_detected_callback(const px4_msgs::msg::VehicleLandDetected::UniquePtr msg)
    {
        // if the drone landed (msg->landed), reset the trigger.
        // The flag landing_triggered_ ensures we only reset when we had previously forced a landing.
        if (msg->landed && landing_triggered_)
        {
            std::cout << "Drone landed successfully. Resetting force land trigger." << std::endl;
            landing_triggered_ = false;
            need_land = false;
        }
    }

    void height_callback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg)
    {
        float z_ = -msg->z;

        std::cout << "Current drone height: " << z_ << " meters" << std::endl;

        // Check if height exceeds 20 meters and landing hasn't been triggered yet
        if (z_ > 20.0 && !landing_triggered_)
        {
            need_land = true;
        }

        return;
    }

    void activate_switch()
    {
        if (need_land)
        {
            std::cout << "Drone height exceeded 20 meters threshold, Landing forced!" << std::endl;

            auto command = px4_msgs::msg::VehicleCommand();
            command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;

            command.target_system = 1;
            command.target_component = 1;
            command.source_system = 1;
            command.source_component = 1;
            command.from_external = true;
            command.timestamp = this->get_clock()->now().nanoseconds() / 1000;

            this->publisher_->publish(command);

            need_land = false;
            landing_triggered_ = true;
        }
    }
};

int main(int argc, char *argv[])
{
    std::cout << "Starting vehicle_local_position listener node..." << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForceLand>());
    rclcpp::shutdown();
    return 0;
}