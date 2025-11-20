#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <vector>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <offboard_rl/utils.h>
#include <unsupported/Eigen/Splines>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class ExecuteTrajectory : public rclcpp::Node
{
public:
    ExecuteTrajectory() : Node("execute_trajectory")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // Subscribers
        local_position_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position",
            qos, std::bind(&ExecuteTrajectory::vehicle_local_position_callback, this, std::placeholders::_1));

        attitude_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude",
            qos, std::bind(&ExecuteTrajectory::vehicle_attitude_callback, this, std::placeholders::_1));

        // Publishers
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);

        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);

        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        timer_offboard_ = this->create_wall_timer(100ms, std::bind(&ExecuteTrajectory::activate_offboard, this));
        timer_trajectory_publish_ = this->create_wall_timer(20ms, std::bind(&ExecuteTrajectory::publish_trajectory_setpoint, this));
    }

private:
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_subscription_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_subscription_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;

    rclcpp::TimerBase::SharedPtr timer_offboard_;
    rclcpp::TimerBase::SharedPtr timer_trajectory_publish_;

    bool offboard_active{false};
    bool trajectory_generated{false};
    bool landing_requested_{false}; // Flag to manage landing at the end of the mission

    // Temporal Parameters
    double total_time_ = 40.0;  // Total time to traverse the spline (seconds)
    double t_elapsed_ = 0.0;    // Elapsed time
    double offboard_counter{0}; // Counter for arming delay

    // Vehicle Data
    VehicleLocalPosition current_position_{};
    VehicleAttitude current_attitude_{};

    // 4 Dimension Splines (X, Y, Z, Yaw) di grado 3 (Cubica)
    typedef Eigen::Spline<double, 4> Spline4d;
    Spline4d spline_;

    void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        current_position_ = *msg;
    }

    void vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
    {
        current_attitude_ = *msg;
    }

    // Function to generate the 7 waypoints and interpolate them with the B-Spline
    void generate_spline_trajectory()
    {
        // 1. Create a matrix of points: 4 rows (x,y,z,yaw) x 7 columns (waypoints)
        Eigen::MatrixXd points(4, 7);

        // Calculate initial Yaw for relative references
        double start_yaw = utilities::quatToRpy(Eigen::Vector4d(
            current_attitude_.q[0], current_attitude_.q[1],
            current_attitude_.q[2], current_attitude_.q[3]))(2);

        // Point 0
        points.col(0) << current_position_.x, current_position_.y, current_position_.z, start_yaw;

        // Point 1
        points.col(1) << current_position_.x + 10.0, current_position_.y + 2.0, -5.0, start_yaw;

        // Point 2
        points.col(2) << current_position_.x + 7.0, current_position_.y + 15.0, -10.0, start_yaw + 0.5;

        // Point 3
        points.col(3) << current_position_.x + 8.0, current_position_.y + 10.0, -18.0, start_yaw + 1.5;

        // Point 4
        points.col(4) << current_position_.x - 10.0, current_position_.y - 14.0, -6.0, start_yaw + 2.5;

        // Point 5
        points.col(5) << current_position_.x - 15.0, current_position_.y - 13.0, -6.0, start_yaw + 3.0;

        // Point 6
        points.col(6) << current_position_.x + 0.0, current_position_.y + 0.0, -5.0, start_yaw + 3.14;

        // Interpolate the points with a B-Spline of degree 3
        // "Interpolate" creates a curve that passes exactly through all points.
        spline_ = Eigen::SplineFitting<Spline4d>::Interpolate(points, 3);

        trajectory_generated = true;
        t_elapsed_ = 0.0;

        std::cout << "B-Spline Trajectory Generated with 7 waypoints!" << std::endl;
    }

    void activate_offboard()
    {
        // If already landing, do nothing
        if (landing_requested_)
        {
            return;
        }

        // Arming
        if (offboard_counter == 10)
        {

            // Generate trajectory if not already done
            if (!trajectory_generated)
            {
                generate_spline_trajectory();
            }

            // Command: Set Offboard mode
            VehicleCommand msg{};
            msg.param1 = 1; // base_mode
            msg.param2 = 6; // custom_main_mode = OFFBOARD
            msg.command = VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
            msg.target_system = 1;
            msg.target_component = 1;
            msg.source_system = 1;
            msg.source_component = 1;
            msg.from_external = true;
            msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            vehicle_command_publisher_->publish(msg);

            // Command: Arm Vehicle
            msg.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
            msg.param1 = 1.0; // Arm
            vehicle_command_publisher_->publish(msg);

            offboard_active = true;
            std::cout << "Offboard activated & Vehicle Armed." << std::endl;
        }

        OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = true;
        msg.acceleration = true;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);

        if (offboard_counter < 11)
            offboard_counter++;
    }

    void publish_trajectory_setpoint()
    {
        if (!offboard_active || !trajectory_generated || landing_requested_)
        {
            return;
        }

        double u = t_elapsed_ / total_time_;

        // Check if trajectory is finished than trigger LAND command
        if (u >= 1.0)
        {
            std::cout << "Trajectory finished (u=1.0). Triggering LAND command..." << std::endl;

            VehicleCommand msg{};
            msg.command = VehicleCommand::VEHICLE_CMD_NAV_LAND;
            msg.target_system = 1;
            msg.target_component = 1;
            msg.source_system = 1;
            msg.source_component = 1;
            msg.from_external = true;
            msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            vehicle_command_publisher_->publish(msg);

            landing_requested_ = true;
            return;
        }

        auto values = spline_.derivatives(u, 2);

        // vel(t) = spline'(u) * (du/dt)  -> du/dt = 1/TotalTime
        // acc(t) = spline''(u) * (du/dt)^2
        double dt_inv = 1.0 / total_time_;
        double dt2_inv = dt_inv * dt_inv;

        Eigen::Vector4d pos = values.col(0);
        Eigen::Vector4d vel = values.col(1) * dt_inv;
        Eigen::Vector4d acc = values.col(2) * dt2_inv;

        TrajectorySetpoint msg{};

        msg.position = {float(pos(0)), float(pos(1)), float(pos(2))};
        msg.velocity = {float(vel(0)), float(vel(1)), float(vel(2))};
        msg.acceleration = {float(acc(0)), float(acc(1)), float(acc(2))};
        msg.yaw = float(pos(3));

        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);

        // Time update
        double dt_loop = 0.02;
        t_elapsed_ += dt_loop;
    }
};

int main(int argc, char *argv[])
{
    std::cout << "Starting Spline Trajectory Node..." << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExecuteTrajectory>());
    rclcpp::shutdown();
    return 0;
}
