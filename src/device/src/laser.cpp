#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <modbus/modbus.h>
#include <chrono>
#include <vector>
#include <string>

class LaserNode : public rclcpp::Node
{
public:
    LaserNode() : Node("laser_node")
    {
        // Parameters
        port_ = this->declare_parameter("port", std::string("/dev/ttyAMA0"));
        slave_id_ = this->declare_parameter("slave_id", 1);
        baudrate_ = this->declare_parameter("baudrate", 115200);
        frequency_ = this->declare_parameter("frequency", 50.0);
        
        // Initialize Modbus
        if (!init_modbus()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize Modbus");
            return;
        }
        
        // Publisher
        laser_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/device/laser/distances", 1);
        
        // Timer for reading laser data
        int period_ms = static_cast<int>(1000.0 / frequency_);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&LaserNode::read_laser_data, this));
        
        RCLCPP_INFO(this->get_logger(), "Laser Node initialized on %s at %d baud, %d Hz", 
                    port_.c_str(), baudrate_, static_cast<int>(frequency_));
    }
    
    ~LaserNode()
    {
        if (ctx_) {
            modbus_close(ctx_);
            modbus_free(ctx_);
        }
    }

private:
    bool init_modbus()
    {
        // Create Modbus RTU context
        ctx_ = modbus_new_rtu(port_.c_str(), baudrate_, 'N', 8, 1);
        if (!ctx_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create Modbus RTU context");
            return false;
        }
        
        // Set slave ID
        if (modbus_set_slave(ctx_, slave_id_) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set slave ID: %s", modbus_strerror(errno));
            modbus_free(ctx_);
            ctx_ = nullptr;
            return false;
        }
        
        // Set timeout
        modbus_set_response_timeout(ctx_, 1, 0); // 1 second timeout
        
        // Enable debug if needed
        // modbus_set_debug(ctx_, TRUE);
        
        // Connect
        if (modbus_connect(ctx_) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect: %s", modbus_strerror(errno));
            modbus_free(ctx_);
            ctx_ = nullptr;
            return false;
        }
        
        // Flush serial buffers
        modbus_flush(ctx_);
        
        RCLCPP_INFO(this->get_logger(), "Modbus connected successfully");
        return true;
    }
    
    void read_laser_data()
    {
        if (!ctx_) {
            RCLCPP_ERROR(this->get_logger(), "Modbus context not initialized");
            return;
        }
        
        uint16_t registers[5];
        
        // Read 5 registers starting from address 0
        int result = modbus_read_registers(ctx_, 0, 5, registers);
        
        if (result == -1) {
            error_count_++;
            if (error_count_ % 10 == 1) { // Log every 10th error to avoid spam
                RCLCPP_WARN(this->get_logger(), "Failed to read registers: %s (error #%d)", 
                           modbus_strerror(errno), error_count_);
            }
            
            // Try to reconnect if too many consecutive errors
            if (error_count_ > 50) {
                RCLCPP_WARN(this->get_logger(), "Too many errors, attempting reconnection...");
                reconnect();
                error_count_ = 0;
            }
            return;
        }
        
        // Reset error count on successful read
        error_count_ = 0;
        
        // Prepare message
        std_msgs::msg::Float32MultiArray msg;
        msg.data.resize(5);
        
        // Convert to cm and populate message
        for (int i = 0; i < 5; ++i) {
            msg.data[i] = static_cast<float>(registers[i]); // Assuming values are already in cm
        }
        
        // Publish
        laser_pub_->publish(msg);
        
        // Debug output (reduce frequency to avoid spam)
        static int debug_counter = 0;
        if (++debug_counter >= 25) { // Every 25 readings (0.5s at 50Hz)
            debug_counter = 0;
            RCLCPP_INFO(this->get_logger(), 
                       "Probe 1: %.0f cm | Probe 2: %.0f cm | Probe 3: %.0f cm | Probe 4: %.0f cm | Probe 5: %.0f cm",
                       msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4]);
        }
    }
    
    void reconnect()
    {
        if (ctx_) {
            modbus_close(ctx_);
            
            // Wait a bit before reconnecting
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            if (modbus_connect(ctx_) == -1) {
                RCLCPP_ERROR(this->get_logger(), "Reconnection failed: %s", modbus_strerror(errno));
            } else {
                modbus_flush(ctx_);
                RCLCPP_INFO(this->get_logger(), "Reconnected successfully");
            }
        }
    }
    
    // Parameters
    std::string port_;
    int slave_id_;
    int baudrate_;
    double frequency_;
    
    // Modbus
    modbus_t* ctx_;
    int error_count_ = 0;
    
    // ROS2 interfaces
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr laser_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}