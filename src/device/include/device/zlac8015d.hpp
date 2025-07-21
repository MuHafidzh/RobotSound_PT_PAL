#ifndef ZLAC8015D_HPP
#define ZLAC8015D_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <chrono>
#include <vector>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <thread>
#include <cmath>

class ZLAC8015Node : public rclcpp::Node
{
public:
    ZLAC8015Node();
    ~ZLAC8015Node();

private:
    // CAN functions
    bool init_can();
    void send_can_message(const std::vector<uint8_t>& data);
    void read_can_messages();
    void process_can_response(const struct can_frame& frame);
    
    // Motor control functions
    void set_speed(int axis, int speed);
    void request_encoder(int axis);
    void request_vbus();
    void enable_motor();
    void disable_motor();
    
    // Kinematics
    void diff_drive_kinematics(double vx, double wz, double& v_left, double& v_right);
    int ms_to_rpm(double v);
    
    // ROS callbacks
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void control_loop();
    void calculate_actual_rpm();
    void publish_data();
    
    // Parameters
    double wheel_base_;
    double wheel_radius_;
    int encoder_cpr_;
    std::string can_interface_;
    
    // CAN
    int can_socket_;
    
    // Motor state
    std::vector<int32_t> enc_;
    std::vector<int32_t> prev_enc_;
    std::vector<double> actual_rpm_;
    double vbus_;
    bool driver_enabled_;
    
    // Control
    double vx_ = 0.0;
    double wz_ = 0.0;
    std::chrono::steady_clock::time_point prev_time_;
    
    // ROS2 interfaces
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr rpm_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vbus_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr can_timer_;
};

#endif // ZLAC8015D_HPP