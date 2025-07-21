//this is gpio node for raspberry pi 5
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <gpiod.hpp>
#include <chrono>
#include <vector>

class GpioNode : public rclcpp::Node
{
public:
    GpioNode() : Node("gpio_reader_node")
    {
        // GPIO pins to monitor (same as your Python code)
        pin_numbers_ = {12, 5, 16, 6}; //a b c d
        
        // Initialize debouncing vectors
        last_raw_states_.resize(pin_numbers_.size(), 0);
        stable_states_.resize(pin_numbers_.size(), 0);
        last_change_times_.resize(pin_numbers_.size());
        auto now = std::chrono::steady_clock::now();
        std::fill(last_change_times_.begin(), last_change_times_.end(), now);
        
        // Initialize GPIO chip and lines
        try {
            RCLCPP_INFO(this->get_logger(), "Initializing gpiochip4...");
            chip_ = gpiod::chip("gpiochip4");
            
            // Request input lines with pull-up
            for (int pin : pin_numbers_) {
                auto line = chip_.get_line(pin);
                line.request({"gpio_reader", gpiod::line_request::DIRECTION_INPUT, gpiod::line_request::FLAG_BIAS_PULL_UP});
                lines_.push_back(std::move(line));
            }
            
            RCLCPP_INFO(this->get_logger(), "Successfully initialized gpiochip4");
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize GPIO: %s", e.what());
            return;
        }
        
        // Create publisher for GPIO states
        publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/device/gpio", 1);
        
        // Create timer to read GPIO states every 100ms for debugging
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&GpioNode::read_gpio_callback, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "GPIO Reader Node started, monitoring pins: 5, 6, 12, 16");
    }
    
    ~GpioNode()
    {
        RCLCPP_INFO(this->get_logger(), "GPIO Reader Node shutting down");
    }

private:
    void read_gpio_callback()
    {
        auto current_time = std::chrono::steady_clock::now();
        
        // Debug output every 5 seconds
        static auto last_debug = std::chrono::steady_clock::now();
        bool show_debug = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_debug).count() >= 5;
        if (show_debug) last_debug = current_time;
        
        for (size_t i = 0; i < pin_numbers_.size(); ++i) {
            try {
                int raw_state = lines_[i].get_value();
                int pin = pin_numbers_[i];
                
                if (show_debug) {
                    RCLCPP_INFO(this->get_logger(), "DEBUG - GPIO %d raw value: %d", pin, raw_state);
                }
                
                // Debouncing logic
                if (raw_state != last_raw_states_[i]) {
                    last_change_times_[i] = current_time;
                    last_raw_states_[i] = raw_state;
                }
                
                auto time_since_change = std::chrono::duration_cast<std::chrono::milliseconds>(
                    current_time - last_change_times_[i]).count();
                
                if (time_since_change >= debounce_delay_ms_) {
                    if (stable_states_[i] != raw_state) {
                        stable_states_[i] = raw_state;
                        RCLCPP_INFO(this->get_logger(), "GPIO %d: %s", pin, raw_state ? "HIGH" : "LOW");
                    }
                }
                
            }
            catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error reading GPIO %d: %s", pin_numbers_[i], e.what());
            }
        }
        
        // Publish current states
        std_msgs::msg::UInt8MultiArray msg;
        msg.data.resize(stable_states_.size());
        for (size_t i = 0; i < stable_states_.size(); ++i) {
            msg.data[i] = static_cast<uint8_t>(stable_states_[i]);
        }
        publisher_->publish(msg);
    }
    
    std::vector<int> pin_numbers_;
    std::vector<gpiod::line> lines_;
    gpiod::chip chip_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Debouncing variables
    std::vector<int> last_raw_states_;
    std::vector<int> stable_states_;
    std::vector<std::chrono::steady_clock::time_point> last_change_times_;
    const int debounce_delay_ms_ = 50;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GpioNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}