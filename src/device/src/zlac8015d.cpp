#include "device/zlac8015d.hpp"

ZLAC8015Node::ZLAC8015Node() : Node("zlac8015_node")
{
    // Robot parameters
    wheel_base_ = this->declare_parameter("wheel_base", 0.3);
    wheel_radius_ = this->declare_parameter("wheel_radius", 0.107);
    encoder_cpr_ = this->declare_parameter("encoder_cpr", 4096);
    can_interface_ = this->declare_parameter("can_interface", std::string("can0"));
    
    // Initialize CAN
    if (!init_can()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN interface");
        return;
    }
    
    // Initialize variables
    enc_ = {0, 0};
    prev_enc_ = {0, 0};
    actual_rpm_ = {0.0, 0.0};
    vbus_ = 0.0;
    driver_enabled_ = false;
    prev_time_ = std::chrono::steady_clock::now();
    
    // Publishers
    encoder_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/device/motor/encoder", 1);
    rpm_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/device/motor/rpm", 1);
    vbus_pub_ = this->create_publisher<std_msgs::msg::Float32>("/device/motor/vbus", 1);
    
    // Subscribers
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/device/motor/cmd_vel", 1, std::bind(&ZLAC8015Node::cmd_vel_callback, this, std::placeholders::_1));
    
    // Timer for main control loop
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ZLAC8015Node::control_loop, this));
    
    // Timer for CAN reading
    can_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&ZLAC8015Node::read_can_messages, this));
    
    RCLCPP_INFO(this->get_logger(), "ZLAC8015D Node initialized");
}

ZLAC8015Node::~ZLAC8015Node()
{
    if (driver_enabled_) {
        disable_motor();
    }
    if (can_socket_ >= 0) {
        close(can_socket_);
    }
}

bool ZLAC8015Node::init_can()
{
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create CAN socket");
        return false;
    }
    
    struct ifreq ifr;
    strcpy(ifr.ifr_name, can_interface_.c_str());
    ioctl(can_socket_, SIOCGIFINDEX, &ifr);
    
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to bind CAN socket");
        close(can_socket_);
        return false;
    }
    
    // Set non-blocking
    int flags = fcntl(can_socket_, F_GETFL, 0);
    fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK);
    
    RCLCPP_INFO(this->get_logger(), "CAN interface %s initialized", can_interface_.c_str());
    return true;
}

void ZLAC8015Node::send_can_message(const std::vector<uint8_t>& data)
{
    struct can_frame frame;
    frame.can_id = 0x601;  // TX ID
    frame.can_dlc = 8;
    
    for (size_t i = 0; i < 8; ++i) {
        frame.data[i] = (i < data.size()) ? data[i] : 0x00;
    }
    
    write(can_socket_, &frame, sizeof(struct can_frame));
}

void ZLAC8015Node::set_speed(int axis, int speed)
{
    uint8_t index = 0x01 + axis;
    std::vector<uint8_t> data = {0x23, 0xFF, 0x60, index};
    
    // Pack speed as little-endian 32-bit integer
    data.push_back(speed & 0xFF);
    data.push_back((speed >> 8) & 0xFF);
    data.push_back((speed >> 16) & 0xFF);
    data.push_back((speed >> 24) & 0xFF);
    
    send_can_message(data);
}

void ZLAC8015Node::request_encoder(int axis)
{
    uint8_t index = 0x01 + axis;
    std::vector<uint8_t> data = {0x43, 0x64, 0x60, index, 0x00, 0x00, 0x00, 0x00};
    send_can_message(data);
}

void ZLAC8015Node::request_vbus()
{
    std::vector<uint8_t> data = {0x4B, 0x35, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};
    send_can_message(data);
}

void ZLAC8015Node::enable_motor()
{
    std::vector<uint8_t> commands = {0x00, 0x06, 0x07, 0x0F};
    for (uint8_t cmd : commands) {
        std::vector<uint8_t> data = {0x2B, 0x40, 0x60, 0x00, cmd, 0x00, 0x00, 0x00};
        send_can_message(data);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    driver_enabled_ = true;
    RCLCPP_INFO(this->get_logger(), "Motor enabled");
}

void ZLAC8015Node::disable_motor()
{
    std::vector<uint8_t> data = {0x2B, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
    send_can_message(data);
    driver_enabled_ = false;
    RCLCPP_INFO(this->get_logger(), "Motor disabled");
}

void ZLAC8015Node::diff_drive_kinematics(double vx, double wz, double& v_left, double& v_right)
{
    v_left = vx - (wz * wheel_base_ / 2.0);
    v_right = vx + (wz * wheel_base_ / 2.0);
    v_right = -v_right;  // Invert right wheel
}

int ZLAC8015Node::ms_to_rpm(double v)
{
    return static_cast<int>((v / (2.0 * M_PI * wheel_radius_)) * 60.0);
}

void ZLAC8015Node::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    vx_ = msg->linear.x;
    wz_ = msg->angular.z;
}

void ZLAC8015Node::read_can_messages()
{
    struct can_frame frame;
    while (read(can_socket_, &frame, sizeof(struct can_frame)) > 0) {
        if (frame.can_id == 0x581) {  // RX ID
            process_can_response(frame);
        }
    }
}

void ZLAC8015Node::process_can_response(const struct can_frame& frame)
{
    if (frame.can_dlc >= 8) {
        // Check encoder responses
        if (frame.data[0] == 0x43 && frame.data[1] == 0x64 && frame.data[2] == 0x60) {
            if (frame.data[3] == 0x01) {
                // Encoder 0
                enc_[0] = *reinterpret_cast<const int32_t*>(&frame.data[4]);
            } else if (frame.data[3] == 0x02) {
                // Encoder 1
                enc_[1] = *reinterpret_cast<const int32_t*>(&frame.data[4]);
            }
        }
        // Check VBUS response
        else if (frame.data[0] == 0x4B && frame.data[1] == 0x35 && frame.data[2] == 0x20 && frame.data[3] == 0x00) {
            uint16_t vbus_raw = *reinterpret_cast<const uint16_t*>(&frame.data[4]);
            vbus_ = vbus_raw * 0.01;
        }
    }
}

void ZLAC8015Node::control_loop()
{
    // Calculate wheel velocities
    double v_left, v_right;
    diff_drive_kinematics(vx_, wz_, v_left, v_right);
    int rpm_left = ms_to_rpm(v_left);
    int rpm_right = ms_to_rpm(v_right);
    
    bool should_enable = !(v_left == 0.0 && v_right == 0.0);

    // Add CAN-aware state management
    static auto last_state_change = std::chrono::steady_clock::now();
    static bool waiting_for_ack = false;
    static int retry_count = 0;
    auto current_time = std::chrono::steady_clock::now();
    auto time_since_change = std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - last_state_change).count();

    if (should_enable != driver_enabled_) {
        if (!waiting_for_ack && time_since_change >= 200) {  // 200ms debounce
            waiting_for_ack = true;
            last_state_change = current_time;
            retry_count = 0;
            
            if (should_enable) {
                RCLCPP_INFO(this->get_logger(), "Enabling motor...");
                enable_motor();
            } else {
                RCLCPP_INFO(this->get_logger(), "Disabling motor - stopping first...");
                // Critical: Stop motors before disable
                set_speed(0, 0);
                set_speed(1, 0);
                std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow CAN to process
                disable_motor();
            }
        }
        // Timeout handling - if no response after 1 second, retry
        else if (waiting_for_ack && time_since_change >= 1000) {
            retry_count++;
            if (retry_count < 3) {
                RCLCPP_WARN(this->get_logger(), "State change timeout, retrying... (%d/3)", retry_count);
                waiting_for_ack = false; // Allow retry
            } else {
                RCLCPP_ERROR(this->get_logger(), "Motor state change failed after 3 retries");
                waiting_for_ack = false;
                retry_count = 0;
            }
        }
    }
    // Reset ACK waiting when state matches
    else if (waiting_for_ack) {
        waiting_for_ack = false;
        retry_count = 0;
        RCLCPP_INFO(this->get_logger(), "Motor state change confirmed");
    }

    // Only send commands if motor is truly enabled
    if (driver_enabled_ && !waiting_for_ack) {        
        set_speed(0, rpm_left);
        set_speed(1, rpm_right);
    }
    // // Check if should enable/disable motor
    // bool should_enable = !(v_left == 0.0 && v_right == 0.0);
    // if (should_enable != driver_enabled_) {
    //     if (should_enable) {
    //         enable_motor();
    //     } else {
    //         disable_motor();
    //     }
    // }
    
    // // Convert to RPM and send commands
    // int rpm_left = ms_to_rpm(v_left);
    // int rpm_right = ms_to_rpm(v_right);
    
    // set_speed(0, rpm_left);
    // set_speed(1, rpm_right);
    
    // Request encoder and VBUS data
    request_encoder(0);
    request_encoder(1);
    request_vbus();
    
    // Calculate actual RPM from encoders
    calculate_actual_rpm();
    
    // Publish data
    publish_data();
    
    // Debug output
    RCLCPP_INFO(this->get_logger(), 
        "ENC0: %d | ENC1: %d | VBUS: %.2fV | RPM L: %d | RPM R: %d | ACT RPM L: %.2f | ACT RPM R: %.2f",
        enc_[0], enc_[1], vbus_, rpm_left, rpm_right, actual_rpm_[0], actual_rpm_[1]);
}

void ZLAC8015Node::calculate_actual_rpm()
{
    auto current_time = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(current_time - prev_time_).count();
    
    if (dt > 0) {
        for (int i = 0; i < 2; ++i) {
            int32_t delta = enc_[i] - prev_enc_[i];
            actual_rpm_[i] = (static_cast<double>(delta) / encoder_cpr_) / dt * 60.0;
            prev_enc_[i] = enc_[i];
        }
        prev_time_ = current_time;
    }
}

void ZLAC8015Node::publish_data()
{
    // Publish encoders
    std_msgs::msg::Int32MultiArray enc_msg;
    enc_msg.data = {enc_[0], enc_[1]};
    encoder_pub_->publish(enc_msg);
    
    // Publish RPM
    std_msgs::msg::Float32MultiArray rpm_msg;
    rpm_msg.data = {static_cast<float>(actual_rpm_[0]), static_cast<float>(actual_rpm_[1])};
    rpm_pub_->publish(rpm_msg);
    
    // Publish VBUS
    std_msgs::msg::Float32 vbus_msg;
    vbus_msg.data = vbus_;
    vbus_pub_->publish(vbus_msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ZLAC8015Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}