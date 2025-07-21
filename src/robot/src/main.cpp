#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <vector>
#include <string>
#include <sstream>

enum class RobotState {
    IDLE,
    MANUAL,
    AUTO
};

enum class AutoState {
    FOLLOWING,  // Following detected person
    SEARCHING,  // Lost person, searching
    STOPPED     // No person found, waiting
};

class RobotMainNode : public rclcpp::Node
{
public:
    RobotMainNode() : Node("robot_main_node")
    {
        // Initialize state
        current_state_ = RobotState::IDLE;
        auto_state_ = AutoState::STOPPED;
        
        // Initialize GPIO states
        gpio_states_ = {0, 0, 0, 0}; // a, b, c, d
        prev_gpio_states_ = {0, 0, 0, 0}; // previous states for edge detection
        
        // Initialize manual movement states (toggle states)
        manual_forward_ = false;
        manual_backward_ = false;
        manual_left_ = false;
        manual_right_ = false;
        
        // Initialize other sensor data
        laser_distances_ = {0.0, 0.0, 0.0, 0.0, 0.0};
        motor_encoders_ = {0, 0};
        motor_rpm_ = {0.0, 0.0};
        motor_vbus_ = 0.0;
        
        // Manual control velocities - constant speeds
        linear_speed_ = 0.3;   // m/s
        angular_speed_ = 2.0;  // rad/s
        
        // Auto mode parameters (could be moved to launch file)
        target_distance_ = this->declare_parameter("auto.target_distance", 1.0);  // meters
        min_distance_ = this->declare_parameter("auto.min_distance", 0.5);        // meters
        max_linear_speed_ = this->declare_parameter("auto.max_linear_speed", 1.2); // m/s
        max_angular_speed_ = this->declare_parameter("auto.max_angular_speed", 2.5); // rad/s
        detection_timeout_ = this->declare_parameter("auto.detection_timeout", 10.0); // seconds
        search_duration_ = this->declare_parameter("auto.search_duration", 16.0);   // seconds
        search_angular_speed_ = this->declare_parameter("auto.search_angular_speed", 2.0); // rad/s
        
        // Tambahkan gain parameters
        angular_gain_ = this->declare_parameter("auto.angular_gain", 0.002);  // person tracking gain
        linear_gain_ = this->declare_parameter("auto.linear_gain", 0.005);    // distance control gain
        control_threshold_cm_ = this->declare_parameter("auto.control_threshold_cm", 5.0); // ±5cm dead zone
        max_control_distance_cm_ = this->declare_parameter("auto.max_control_distance_cm", 400.0); // 400cm max

        // Camera parameters for person tracking
        camera_width_ = this->declare_parameter("camera.width", 480);
        camera_height_ = this->declare_parameter("camera.height", 360);
        
        // Initialize auto mode variables
        last_detection_time_ = this->now();
        search_start_time_ = this->now();
        search_direction_ = 1; // 1 for right, -1 for left
        person_detected_ = false;
        person_x_ = 0.0;
        person_y_ = 0.0;
        person_confidence_ = 0.0;
        
        // Subscribers
        gpio_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/device/gpio", 1,
            std::bind(&RobotMainNode::gpio_callback, this, std::placeholders::_1));
            
        yolo_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/camera/detections", 1,
            std::bind(&RobotMainNode::yolo_callback, this, std::placeholders::_1));
            
        laser_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/device/laser/distances", 1,
            std::bind(&RobotMainNode::laser_callback, this, std::placeholders::_1));
            
        encoder_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/device/motor/encoder", 1,
            std::bind(&RobotMainNode::encoder_callback, this, std::placeholders::_1));
            
        rpm_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/device/motor/rpm", 1,
            std::bind(&RobotMainNode::rpm_callback, this, std::placeholders::_1));
            
        vbus_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/device/motor/vbus", 1,
            std::bind(&RobotMainNode::vbus_callback, this, std::placeholders::_1));
        
        // Publisher for motor commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/device/motor/cmd_vel", 10);
        
        // Main control timer (50Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&RobotMainNode::control_loop, this));
        
        // Status timer (1Hz for logging)
        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&RobotMainNode::status_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Robot Main Node initialized in IDLE state");
        RCLCPP_INFO(this->get_logger(), "Controls: A+B=Manual | C+D=Idle | B+D=Auto");
        RCLCPP_INFO(this->get_logger(), "Manual: A=Forward | B=Backward | C=Left | D=Right (TOGGLE)");
        RCLCPP_INFO(this->get_logger(), "Auto Mode Parameters:");
        RCLCPP_INFO(this->get_logger(), "  Target Distance: %.2f m", target_distance_);
        RCLCPP_INFO(this->get_logger(), "  Min Distance: %.2f m", min_distance_);
        RCLCPP_INFO(this->get_logger(), "  Max Linear Speed: %.2f m/s", max_linear_speed_);
        RCLCPP_INFO(this->get_logger(), "  Max Angular Speed: %.2f rad/s", max_angular_speed_);
        RCLCPP_INFO(this->get_logger(), "  Detection Timeout: %.2f seconds", detection_timeout_);
        RCLCPP_INFO(this->get_logger(), "  Search Duration: %.2f seconds", search_duration_);
        RCLCPP_INFO(this->get_logger(), "  Search Angular Speed: %.2f rad/s", search_angular_speed_);
        RCLCPP_INFO(this->get_logger(), "  Angular Gain: %.4f", angular_gain_);
        RCLCPP_INFO(this->get_logger(), "  Linear Gain: %.4f", linear_gain_);
        RCLCPP_INFO(this->get_logger(), "  Control Threshold: %.1f cm", control_threshold_cm_);
        RCLCPP_INFO(this->get_logger(), "  Max Control Distance: %.1f cm", max_control_distance_cm_);
    }

private:
    void gpio_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
        if (msg->data.size() >= 4) {
            prev_gpio_states_ = gpio_states_;
            gpio_states_ = {msg->data[0], msg->data[1], msg->data[2], msg->data[3]}; // a, b, c, d
            
            check_state_transitions();
            
            // Handle manual control button presses (only in manual mode)
            if (current_state_ == RobotState::MANUAL) {
                handle_manual_button_presses();
            }
        }
    }
    
    void yolo_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (!msg->data.empty()) {
            // Parse YOLO detection data
            // Format: "person_blue:confidence:x:y:width:height"
            if (parse_detection_data(msg->data)) {
                person_detected_ = true;
                last_detection_time_ = this->now(); // Update detection time
                
                // State transitions based on current auto state
                if (current_state_ == RobotState::AUTO) {
                    switch (auto_state_) {
                        case AutoState::STOPPED:
                            auto_state_ = AutoState::FOLLOWING;
                            RCLCPP_INFO(this->get_logger(), "Person detected! Switching to FOLLOWING mode");
                            break;
                        case AutoState::SEARCHING:
                            auto_state_ = AutoState::FOLLOWING;
                            RCLCPP_INFO(this->get_logger(), "Person found during search! Switching to FOLLOWING mode");
                            break;
                        case AutoState::FOLLOWING:
                            // Already following, just continue
                            break;
                    }
                }
            } else {
                // Parse failed - treat as no detection
                person_detected_ = false;
            }
        } else {
            // Empty data - no detection
            person_detected_ = false;
        }
        
        // Log detection status untuk debug
        RCLCPP_DEBUG(this->get_logger(), "YOLO callback: person_detected=%s, data_empty=%s, msg='%s'", 
                    person_detected_ ? "true" : "false", 
                    msg->data.empty() ? "true" : "false",
                    msg->data.c_str());
    }
    
    bool parse_detection_data(const std::string& data)
    {
        // Parse "person_blue:confidence:x:y:width:height"
        std::istringstream ss(data);
        std::string token;
        std::vector<std::string> tokens;
        
        while (std::getline(ss, token, ':')) {
            tokens.push_back(token);
        }
        
        if (tokens.size() >= 6 && tokens[0] == "person_blue") {
            try {
                person_confidence_ = std::stof(tokens[1]);
                float x = std::stof(tokens[2]);
                float y = std::stof(tokens[3]);
                float width = std::stof(tokens[4]);
                float height = std::stof(tokens[5]);
                
                // Calculate center point
                person_x_ = x + width / 2.0;
                person_y_ = y + height / 2.0;
                
                return true;
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to parse detection data: %s", e.what());
                return false;
            }
        }
        return false;
    }
    
    void laser_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() >= 5) {
            laser_distances_ = msg->data;
        }
    }
    
    void encoder_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() >= 2) {
            motor_encoders_ = msg->data;
        }
    }
    
    void rpm_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() >= 2) {
            motor_rpm_ = msg->data;
        }
    }
    
    void vbus_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        motor_vbus_ = msg->data;
    }
    
    void check_state_transitions()
    {
        // Check current button combinations (real-time, high when pressed)
        bool a = gpio_states_[0];  // A button
        bool b = gpio_states_[1];  // B button  
        bool c = gpio_states_[2];  // C button
        bool d = gpio_states_[3];  // D button
        
        // State transitions based on button combinations
        if (a && b) {
            // A+B pressed -> Manual mode
            if (current_state_ != RobotState::MANUAL) {
                change_state(RobotState::MANUAL);
            }
        }
        else if (c && d) {
            // C+D pressed -> Idle mode
            if (current_state_ != RobotState::IDLE) {
                change_state(RobotState::IDLE);
            }
        }
        else if (b && d) {
            // B+D pressed -> Auto mode
            if (current_state_ != RobotState::AUTO) {
                change_state(RobotState::AUTO);
            }
        }
    }
    
    void handle_manual_button_presses()
    {
        // Detect button press events (rising edge: prev=0, current=1)
        bool a_pressed = (gpio_states_[0] && !prev_gpio_states_[0]);
        bool b_pressed = (gpio_states_[1] && !prev_gpio_states_[1]);
        bool c_pressed = (gpio_states_[2] && !prev_gpio_states_[2]);
        bool d_pressed = (gpio_states_[3] && !prev_gpio_states_[3]);
        
        // Toggle movement states on button press
        if (a_pressed) {
            manual_forward_ = !manual_forward_;
            if (manual_forward_) {
                manual_backward_ = false; // Stop backward jika forward aktif
            }
            RCLCPP_INFO(this->get_logger(), "Forward: %s", manual_forward_ ? "ON" : "OFF");
        }
        
        if (b_pressed) {
            manual_backward_ = !manual_backward_;
            if (manual_backward_) {
                manual_forward_ = false; // Stop forward jika backward aktif
            }
            RCLCPP_INFO(this->get_logger(), "Backward: %s", manual_backward_ ? "ON" : "OFF");
        }
        
        if (c_pressed) {
            manual_left_ = !manual_left_;
            if (manual_left_) {
                manual_right_ = false; // Stop right jika left aktif
            }
            RCLCPP_INFO(this->get_logger(), "Left: %s", manual_left_ ? "ON" : "OFF");
        }
        
        if (d_pressed) {
            manual_right_ = !manual_right_;
            if (manual_right_) {
                manual_left_ = false; // Stop left jika right aktif
            }
            RCLCPP_INFO(this->get_logger(), "Right: %s", manual_right_ ? "ON" : "OFF");
        }
    }
    
    void change_state(RobotState new_state)
    {
        current_state_ = new_state;
        
        // Stop robot dan reset manual states when changing states
        stop_robot();
        reset_manual_states();
        
        // Initialize auto state
        if (new_state == RobotState::AUTO) {
            auto_state_ = AutoState::STOPPED;
            search_start_time_ = this->now(); // Initialize timing for STOPPED state
            search_direction_ = 1; // Start search to the right
            RCLCPP_INFO(this->get_logger(), "AUTO mode activated - STOPPED, waiting for person or timeout");
        }
        
        std::string state_name;
        switch (new_state) {
            case RobotState::IDLE:
                state_name = "IDLE";
                break;
            case RobotState::MANUAL:
                state_name = "MANUAL";
                break;
            case RobotState::AUTO:
                state_name = "AUTO-STOPPED";
                break;
        }
        
        RCLCPP_INFO(this->get_logger(), "State changed to: %s", state_name.c_str());
    }
    
    void reset_manual_states()
    {
        manual_forward_ = false;
        manual_backward_ = false;
        manual_left_ = false;
        manual_right_ = false;
    }
    
    void control_loop()
    {
        switch (current_state_) {
            case RobotState::IDLE:
                handle_idle_state();
                break;
            case RobotState::MANUAL:
                handle_manual_state();
                break;
            case RobotState::AUTO:
                handle_auto_state();
                break;
        }
    }
    
    void handle_idle_state()
    {
        // Just stop the robot
        stop_robot();
    }
    
    void handle_manual_state()
    {
        geometry_msgs::msg::Twist cmd_vel;
        
        // Manual control berdasarkan toggle states:
        // Linear movement (maju/mundur)
        if (manual_forward_) {
            cmd_vel.linear.x = linear_speed_;
        }
        else if (manual_backward_) {
            cmd_vel.linear.x = -linear_speed_;
        }
        else {
            cmd_vel.linear.x = 0.0;
        }
        
        // Angular movement (kiri/kanan)
        if (manual_left_) {
            cmd_vel.angular.z = angular_speed_;
        }
        else if (manual_right_) {
            cmd_vel.angular.z = -angular_speed_;
        }
        else {
            cmd_vel.angular.z = 0.0;
        }
        
        // Kombinasi dimungkinkan: forward+left, backward+right, dll
        cmd_vel_pub_->publish(cmd_vel);
    }
    
    void handle_auto_state()
    {
        auto current_time = this->now();
        double time_since_detection = (current_time - last_detection_time_).seconds();
        
        // State machine for auto mode
        switch (auto_state_) {
            case AutoState::FOLLOWING:
                if (person_detected_ && time_since_detection < detection_timeout_) {
                    follow_person();
                } else {
                    // Lost person, STOP and wait for timeout before searching
                    auto_state_ = AutoState::STOPPED;
                    search_start_time_ = current_time; // RESET stopped timer
                    RCLCPP_WARN(this->get_logger(), "Person lost! Stopping and waiting for timeout...");
                    stop_robot(); // Stop immediately when lost
                }
                break;
                
            case AutoState::SEARCHING:
                {
                    // Check if person found during search
                    if (person_detected_ && time_since_detection < detection_timeout_) {
                        // Person found! Switch back to following
                        auto_state_ = AutoState::FOLLOWING;
                        RCLCPP_INFO(this->get_logger(), "Person found during search! Back to following");
                        return; // Exit immediately, don't continue search
                    }
                    
                    // Continue search if within duration
                    double search_time = (current_time - search_start_time_).seconds();
                    if (search_time < search_duration_) {
                        search_for_person();
                    } else {
                        // Search timeout, go back to AUTO::STOPPED (NOT IDLE)
                        RCLCPP_WARN(this->get_logger(), "Search timeout! Back to STOPPED mode");
                        auto_state_ = AutoState::STOPPED;
                        search_start_time_ = current_time; // RESET stopped timer untuk timeout baru
                        stop_robot();
                    }
                }
                break;
                
            case AutoState::STOPPED:
                // Check if person detected to start following
                if (person_detected_ && time_since_detection < detection_timeout_) {
                    auto_state_ = AutoState::FOLLOWING;
                    RCLCPP_INFO(this->get_logger(), "Person detected! Starting following mode");
                } else {
                    // Check if timeout reached to start searching
                    double stopped_time = (current_time - search_start_time_).seconds();
                    if (stopped_time >= detection_timeout_) {
                        // Timeout reached, start searching
                        auto_state_ = AutoState::SEARCHING;
                        search_start_time_ = current_time; // RESET untuk search duration
                        RCLCPP_INFO(this->get_logger(), "Detection timeout! Starting search mode");
                    } else {
                        // Still waiting for timeout, stay stopped
                        stop_robot();
                    }
                }
                break;
        }
    }
    
    void follow_person()
    {
        // VALIDATION: Pastikan person benar-benar terdeteksi
        if (!person_detected_) {
            RCLCPP_WARN(this->get_logger(), "follow_person called but person_detected is false!");
            stop_robot();
            return;
        }
        
        // Validation: Pastikan data detection masih fresh
        auto current_time = this->now();
        double time_since_detection = (current_time - last_detection_time_).seconds();
        if (time_since_detection > detection_timeout_) {
            RCLCPP_WARN(this->get_logger(), "follow_person called but detection is too old (%.1fs)", time_since_detection);
            stop_robot();
            return;
        }
        
        geometry_msgs::msg::Twist cmd_vel;
        
        // Get obstacle distance from center laser (index 2) - convert to cm
        float front_distance_cm = (laser_distances_.size() > 2) ? laser_distances_[2] * 1.0f : 99999.0f;

        // SAFETY: Check sensor samping untuk obstacle (inline)
        float min_side_distance_cm = 99999.0f;
        if (laser_distances_.size() >= 5) {
            // Check sensors 0,1,3,4 (exclude center sensor 2)
            for (int i = 0; i < 5; i++) {
                if (i != 2) { // Skip center sensor
                    float dist_cm = laser_distances_[i] * 1.0f;
                    min_side_distance_cm = std::min(min_side_distance_cm, dist_cm);
                }
            }
        }
        
        // Calculate angular velocity based on person position
        float center_x = static_cast<float>(camera_width_) / 2.0f;
        float error_x = person_x_ - center_x;
        float angular_vel = -static_cast<float>(angular_gain_) * error_x;
        
        // Limit angular velocity (convert double to float untuk calculation)
        float max_ang_speed = static_cast<float>(max_angular_speed_);
        cmd_vel.angular.z = static_cast<double>(std::max(-max_ang_speed, std::min(max_ang_speed, angular_vel)));
        
        // Distance control parameters - SEMUA DALAM CM
        float target_distance_cm = static_cast<float>(target_distance_ * 100.0);  // Convert parameter dari meter ke cm
        float control_threshold = static_cast<float>(control_threshold_cm_);      // Parameter threshold
        float min_safe_distance_cm = static_cast<float>(min_distance_ * 100.0);   // Convert parameter dari meter ke cm
        float max_control_distance = static_cast<float>(max_control_distance_cm_); // Parameter max distance
        
        // Calculate distance error
        float distance_error_cm = front_distance_cm - target_distance_cm; // Positive = too far, negative = too close
        float lin_gain = static_cast<float>(linear_gain_); // Use parameter gain
        
        // Control logic
        // SAFETY FIRST: Check for side obstacles
        if (min_side_distance_cm > 5.0f && min_side_distance_cm < 30.0f) { // 5-30cm safety threshold
            // EMERGENCY: Side obstacles detected
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0; // Stop rotation juga
            printf("SAFETY STOP: Side obstacle detected (%.0fcm)\n", min_side_distance_cm);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "SAFETY STOP: Side obstacle (%.0fcm)", min_side_distance_cm);
        }
        else if (front_distance_cm < min_safe_distance_cm) {
            // TOO CLOSE: Emergency stop
            cmd_vel.linear.x = 0.0;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "EMERGENCY STOP: Too close (%.0fcm)", front_distance_cm);
        }
        else if (front_distance_cm > max_control_distance) {
            // TOO FAR: Stop control (person might be lost)
            cmd_vel.linear.x = 0.0;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "STOP: Person too far (%.0fcm)", front_distance_cm);
        }
        else if (std::abs(distance_error_cm) <= control_threshold) {
            // PERFECT DISTANCE: Within threshold of target, don't move
            cmd_vel.linear.x = 0.0;
        }
        else {
            // PROPORTIONAL CONTROL: Move based on distance error
            float buff_vel = lin_gain * distance_error_cm; // cm * (m/s)/cm = m/s
            
            // Apply speed limits (convert double to float untuk comparison)
            float max_lin_speed = static_cast<float>(max_linear_speed_);
            float limited_vel = std::max(-max_lin_speed, std::min(max_lin_speed, buff_vel));
            cmd_vel.linear.x = static_cast<double>(limited_vel);
        }
        
        // Publish command
        cmd_vel_pub_->publish(cmd_vel);
        
        RCLCPP_DEBUG(this->get_logger(), 
            "Following: dist=%.0fcm, target=%.0fcm, error=%.0fcm, lin=%.3f, ang=%.3f", 
            front_distance_cm, target_distance_cm, distance_error_cm,
            cmd_vel.linear.x, cmd_vel.angular.z);
    }
    
    void search_for_person()
    {
        geometry_msgs::msg::Twist cmd_vel;
        
        auto current_time = this->now();
        double search_time = (current_time - search_start_time_).seconds();
        
        // Change direction every 4 seconds untuk search_duration 16 sec
        // Pattern: 0-4s right, 4-8s left, 8-12s right, 12-16s left
        if (static_cast<int>(search_time) % 8 < 4) {
            cmd_vel.angular.z = search_direction_ * search_angular_speed_;
        } else {
            cmd_vel.angular.z = -search_direction_ * search_angular_speed_;
        }
        
        cmd_vel.linear.x = 0.0; // Don't move forward while searching
        
        cmd_vel_pub_->publish(cmd_vel);
        
        // Debug log
        RCLCPP_DEBUG(this->get_logger(), "Searching: time=%.1fs, direction=%s", 
                    search_time, (cmd_vel.angular.z > 0) ? "RIGHT" : "LEFT");
    }
    
    void stop_robot()
    {
        geometry_msgs::msg::Twist cmd_vel;
        // All velocities are already 0.0 by default
        cmd_vel_pub_->publish(cmd_vel);
    }
    
    void status_loop()
    {
        std::string state_name;
        switch (current_state_) {
            case RobotState::IDLE:
                state_name = "IDLE";
                break;
            case RobotState::MANUAL:
                state_name = "MANUAL";
                break;
            case RobotState::AUTO:
                state_name = "AUTO";
                // Add auto sub-state
                switch (auto_state_) {
                    case AutoState::FOLLOWING:
                        state_name += "-FOLLOWING";
                        break;
                    case AutoState::SEARCHING:
                        state_name += "-SEARCHING";
                        break;
                    case AutoState::STOPPED:
                        state_name += "-STOPPED";
                        break;
                }
                break;
        }
        
        // Auto mode status dengan timing info
        std::string auto_status = "";
        if (current_state_ == RobotState::AUTO) {
            double time_since = (this->now() - last_detection_time_).seconds();
            double timer_time = (this->now() - search_start_time_).seconds();
            
            auto_status = " | Person: ";
            if (person_detected_) {
                auto_status += "DETECTED";
            } else {
                auto_status += "LOST(" + std::to_string(static_cast<int>(time_since)) + "s)";
            }
            auto_status += " | Timer: " + std::to_string(static_cast<int>(timer_time)) + "s";
        }
        
        // Status log dengan manual states info
        std::string manual_status = "";
        if (current_state_ == RobotState::MANUAL) {
            manual_status = " | Manual: ";
            if (manual_forward_) manual_status += "FWD ";
            if (manual_backward_) manual_status += "BWD ";
            if (manual_left_) manual_status += "LEFT ";
            if (manual_right_) manual_status += "RIGHT ";
            if (!manual_forward_ && !manual_backward_ && !manual_left_ && !manual_right_) {
                manual_status += "STOP ";
            }
        }
        
        RCLCPP_INFO(this->get_logger(), 
            "State: %s | GPIO: A=%d B=%d C=%d D=%d%s%s | Laser: [%.1f, %.1f, %.1f, %.1f, %.1f] | VBUS: %.2fV",
            state_name.c_str(),
            gpio_states_[0], gpio_states_[1], gpio_states_[2], gpio_states_[3],
            manual_status.c_str(),
            auto_status.c_str(),
            laser_distances_[0], laser_distances_[1], laser_distances_[2], laser_distances_[3], laser_distances_[4],
            motor_vbus_);
    }
    
    // State variables
    RobotState current_state_;
    AutoState auto_state_;
    
    // GPIO states (a, b, c, d) - current and previous for edge detection
    std::vector<uint8_t> gpio_states_;
    std::vector<uint8_t> prev_gpio_states_;
    
    // Manual control toggle states
    bool manual_forward_;
    bool manual_backward_;
    bool manual_left_;
    bool manual_right_;
    
    // Auto mode parameters
    double target_distance_;
    double min_distance_;
    double max_linear_speed_;
    double max_angular_speed_;
    double detection_timeout_;
    double search_duration_;
    double search_angular_speed_;

    // Tambahkan gain parameters
    double angular_gain_;
    double linear_gain_;
    double control_threshold_cm_;
    double max_control_distance_cm_;

    int camera_width_;
    int camera_height_;
    
    // Auto mode state variables
    rclcpp::Time last_detection_time_;
    rclcpp::Time search_start_time_;
    int search_direction_;
    bool person_detected_;
    float person_x_;
    float person_y_;
    float person_confidence_;
    
    // Sensor data
    std::vector<float> laser_distances_;
    std::vector<int32_t> motor_encoders_;
    std::vector<float> motor_rpm_;
    float motor_vbus_;
    
    // Manual control parameters - constant speeds
    double linear_speed_;
    double angular_speed_;
    
    // ROS2 interfaces
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr gpio_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yolo_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr laser_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr rpm_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vbus_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<RobotMainNode>();
        
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting Robot Main Controller...");
        
        rclcpp::spin(node);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception in main: %s", e.what());
        return -1;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Robot Main Controller shutting down...");
    rclcpp::shutdown();
    return 0;
}