#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/joy_andro.hpp> // Ganti dengan path message Anda jika berbeda
#include <std_msgs/msg/header.hpp>

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

class JoyAndroNode : public rclcpp::Node
{
public:
    JoyAndroNode() : Node("joyandro_node")
    {
        // Inisialisasi state awal
        current_joy_state_.x = 0.0f;
        current_joy_state_.y = 0.0f;
        current_joy_state_.button_x = false;
        current_joy_state_.button_circle = false;
        current_joy_state_.button_triangle = false;
        current_joy_state_.button_square = false;
        running_ = true;

        // Buat publisher untuk topic
        joy_pub_ = this->create_publisher<robot_interfaces::msg::JoyAndro>("/device/joyAndro", 10);

        // Buat timer untuk mempublish state secara periodik (50Hz)
        publisher_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&JoyAndroNode::publish_callback, this));

        // Jalankan loop bluetooth di thread terpisah agar tidak memblokir ROS
        bluetooth_thread_ = std::thread(&JoyAndroNode::bluetooth_loop, this);

        RCLCPP_INFO(this->get_logger(), "JoyAndro Node telah diinisialisasi.");
        RCLCPP_INFO(this->get_logger(), "Mempublish ke topic: /device/joyAndro");
    }

    ~JoyAndroNode()
    {
        running_ = false;
        // Tutup socket untuk membuka blokir 'accept'
        if (server_sock_ >= 0) {
            shutdown(server_sock_, SHUT_RDWR);
            close(server_sock_);
        }
        if (bluetooth_thread_.joinable()) {
            bluetooth_thread_.join();
        }
    }

private:
    // --- Anggota ROS 2 ---
    rclcpp::Publisher<robot_interfaces::msg::JoyAndro>::SharedPtr joy_pub_;
    rclcpp::TimerBase::SharedPtr publisher_timer_;
    robot_interfaces::msg::JoyAndro current_joy_state_;
    std::mutex state_mutex_;

    // --- Anggota Bluetooth & Threading ---
    int server_sock_ = -1;
    int client_sock_ = -1;
    std::thread bluetooth_thread_;
    std::atomic<bool> running_;

    // Loop utama untuk thread Bluetooth
    void bluetooth_loop()
    {
        if (!initialize_bluetooth()) {
            RCLCPP_FATAL(this->get_logger(), "Gagal menginisialisasi Bluetooth. Thread berhenti.");
            return;
        }

        while (running_) {
            RCLCPP_INFO(this->get_logger(), "Menunggu koneksi dari controller di thread Bluetooth...");
            
            struct sockaddr_rc rem_addr = {};
            socklen_t opt = sizeof(rem_addr);
            client_sock_ = accept(server_sock_, (struct sockaddr *)&rem_addr, &opt);

            if (client_sock_ < 0) {
                if (running_) { // Hanya tampilkan error jika kita tidak sedang shutdown
                    RCLCPP_ERROR(this->get_logger(), "Gagal menerima koneksi: %s", strerror(errno));
                }
                continue; // Coba lagi
            }

            char addr_str[19] = {0};
            ba2str(&rem_addr.rc_bdaddr, addr_str);
            RCLCPP_INFO(this->get_logger(), "Koneksi diterima dari: %s", addr_str);

            read_data_from_client();
            close_client_connection();
        }
    }

    // Inisialisasi socket Bluetooth
    bool initialize_bluetooth()
    {
        server_sock_ = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
        if (server_sock_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Gagal membuat socket: %s", strerror(errno));
            return false;
        }

        struct sockaddr_rc loc_addr = {};
        loc_addr.rc_family = AF_BLUETOOTH;
        loc_addr.rc_bdaddr = {{0, 0, 0, 0, 0, 0}};
        loc_addr.rc_channel = (uint8_t)1;

        if (bind(server_sock_, (struct sockaddr *)&loc_addr, sizeof(loc_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Gagal bind socket: %s", strerror(errno));
            return false;
        }

        if (listen(server_sock_, 1) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Gagal listen pada socket: %s", strerror(errno));
            return false;
        }
        return true;
    }

    // Membaca dan mem-parsing data dari klien yang terhubung
    void read_data_from_client() {
        char buffer[1024] = {0};
        std::string partial_data;

        while(running_) {
            memset(buffer, 0, sizeof(buffer));
            int bytes_read = read(client_sock_, buffer, sizeof(buffer) - 1);

            if (bytes_read > 0) {
                partial_data.append(buffer, bytes_read);
                size_t newline_pos;
                while ((newline_pos = partial_data.find('\n')) != std::string::npos) {
                    std::string frame = partial_data.substr(0, newline_pos);
                    partial_data.erase(0, newline_pos + 1);
                    if (!frame.empty()) {
                        parse_and_update_state(frame);
                    }
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Klien terputus (read returned %d, errno: %s)", bytes_read, strerror(errno));
                break;
            }
        }
    }
    
    // Mem-parsing frame data dan memperbarui state
    void parse_and_update_state(const std::string& data) {
        std::istringstream dataStream(data);
        std::string segment;

        std::lock_guard<std::mutex> lock(state_mutex_); // Kunci mutex untuk update state
        while (std::getline(dataStream, segment, ';')) {
            size_t colonPos = segment.find(':');
            if (colonPos == std::string::npos) continue;

            std::string key = segment.substr(0, colonPos);
            std::string value = segment.substr(colonPos + 1);

            try {
                if (key == "joy") {
                    size_t commaPos = value.find(',');
                    if (commaPos != std::string::npos) {
                        current_joy_state_.x = std::stof(value.substr(0, commaPos));
                        current_joy_state_.y = std::stof(value.substr(commaPos + 1));
                    }
                } else {
                    bool pressed = (std::stoi(value) == 1);
                    if (key == "a") current_joy_state_.button_x = pressed;       // Auto -> X
                    else if (key == "m") current_joy_state_.button_circle = pressed; // Manual -> Circle
                    else if (key == "i") current_joy_state_.button_triangle = pressed; // Idle -> Triangle
                    else if (key == "b") current_joy_state_.button_square = pressed;  // Boost -> Square
                }
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Error parsing segment '%s': %s", segment.c_str(), e.what());
            }
        }
    }

    // Callback timer untuk mempublish data
    void publish_callback()
    {
        robot_interfaces::msg::JoyAndro msg_to_publish;
        {
            std::lock_guard<std::mutex> lock(state_mutex_); // Kunci mutex untuk membaca state
            msg_to_publish = current_joy_state_;
        }
        
        // Atur header sebelum publish
        msg_to_publish.header.stamp = this->now();
        msg_to_publish.header.frame_id = "joyandro_link"; // Nama frame yang lebih umum
        
        joy_pub_->publish(msg_to_publish);
    }
    
    void close_client_connection() {
        if (client_sock_ >= 0) {
            close(client_sock_);
            client_sock_ = -1;
            RCLCPP_INFO(this->get_logger(), "Koneksi klien ditutup.");

            // Reset state ke nol saat koneksi terputus
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_joy_state_.x = 0.0f;
            current_joy_state_.y = 0.0f;
            current_joy_state_.button_x = false;
            current_joy_state_.button_circle = false;
            current_joy_state_.button_triangle = false;
            current_joy_state_.button_square = false;
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyAndroNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}