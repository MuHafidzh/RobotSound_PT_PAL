#include <ros/ros.h>
#include <joy_andro/JoyAndro.h> // Path message header di ROS 1
#include <std_msgs/Header.h>

#include <iostream>
#include <string>
#include <sstream>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

class JoyAndroNode
{
public:
    JoyAndroNode(ros::NodeHandle &nh)
    {
        // Inisialisasi state
        current_joy_state_.x = 0.0f;
        current_joy_state_.y = 0.0f;
        current_joy_state_.button_x = false;
        current_joy_state_.button_circle = false;
        current_joy_state_.button_triangle = false;
        current_joy_state_.button_square = false;
        running_ = true;

        // Buat publisher
        joy_pub_ = nh.advertise<joy_andro::JoyAndro>("/device/joyAndro", 10);

        // Buat timer untuk publish (50Hz)
        publisher_timer_ = nh.createTimer(
            ros::Duration(0.02),
            &JoyAndroNode::publish_callback,
            this);
        
        // Jalankan loop bluetooth di thread terpisah
        bluetooth_thread_ = std::thread(&JoyAndroNode::bluetooth_loop, this);

        ROS_INFO("JoyAndro Node ROS 1 telah diinisialisasi.");
    }

    ~JoyAndroNode()
    {
        running_ = false;
        if (server_sock_ >= 0) {
            shutdown(server_sock_, SHUT_RDWR);
            close(server_sock_);
        }
        if (bluetooth_thread_.joinable()) {
            bluetooth_thread_.join();
        }
    }

private:
    // Anggota ROS
    ros::Publisher joy_pub_;
    ros::Timer publisher_timer_;
    joy_andro::JoyAndro current_joy_state_;
    std::mutex state_mutex_;

    // Anggota Bluetooth & Threading
    int server_sock_ = -1;
    int client_sock_ = -1;
    std::thread bluetooth_thread_;
    std::atomic<bool> running_;

    // Fungsi-fungsi inti (sama persis dengan versi ROS 2, hanya loggernya berbeda)
    void bluetooth_loop() {
        if (!initialize_bluetooth()) {
            ROS_FATAL("Gagal menginisialisasi Bluetooth. Thread berhenti.");
            return;
        }

        while (running_ && ros::ok()) {
            ROS_INFO("Menunggu koneksi dari controller...");
            
            struct sockaddr_rc rem_addr = {0};
            socklen_t opt = sizeof(rem_addr);
            client_sock_ = accept(server_sock_, (struct sockaddr *)&rem_addr, &opt);

            if (client_sock_ < 0) {
                if (running_) ROS_ERROR("Gagal menerima koneksi: %s", strerror(errno));
                continue;
            }

            char addr_str[19] = {0};
            ba2str(&rem_addr.rc_bdaddr, addr_str);
            ROS_INFO("Koneksi diterima dari: %s", addr_str);

            read_data_from_client();
            close_client_connection();
        }
    }

    bool initialize_bluetooth() {
        server_sock_ = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
        if (server_sock_ < 0) {
            ROS_ERROR("Gagal membuat socket: %s", strerror(errno));
            return false;
        }
        struct sockaddr_rc loc_addr = {0};
        loc_addr.rc_family = AF_BLUETOOTH;
        loc_addr.rc_bdaddr = {{0, 0, 0, 0, 0, 0}};
        loc_addr.rc_channel = (uint8_t)1;
        if (bind(server_sock_, (struct sockaddr *)&loc_addr, sizeof(loc_addr)) < 0) {
            ROS_ERROR("Gagal bind socket: %s", strerror(errno));
            return false;
        }
        if (listen(server_sock_, 1) < 0) {
            ROS_ERROR("Gagal listen pada socket: %s", strerror(errno));
            return false;
        }
        return true;
    }

    void read_data_from_client() {
        char buffer[1024] = {0};
        std::string partial_data;
        while(running_ && ros::ok()) {
            memset(buffer, 0, sizeof(buffer));
            int bytes_read = read(client_sock_, buffer, sizeof(buffer) - 1);
            if (bytes_read > 0) {
                partial_data.append(buffer, bytes_read);
                size_t newline_pos;
                while ((newline_pos = partial_data.find('\n')) != std::string::npos) {
                    std::string frame = partial_data.substr(0, newline_pos);
                    partial_data.erase(0, newline_pos + 1);
                    if (!frame.empty()) parse_and_update_state(frame);
                }
            } else {
                ROS_WARN("Klien terputus (read returned %d, errno: %s)", bytes_read, strerror(errno));
                break;
            }
        }
    }
    
    void parse_and_update_state(const std::string& data) {
        std::istringstream dataStream(data);
        std::string segment;
        std::lock_guard<std::mutex> lock(state_mutex_);
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
                    if (key == "a") current_joy_state_.button_x = pressed;
                    else if (key == "m") current_joy_state_.button_circle = pressed;
                    else if (key == "i") current_joy_state_.button_triangle = pressed;
                    else if (key == "b") current_joy_state_.button_square = pressed;
                }
            } catch (const std::exception& e) {
                ROS_WARN("Error parsing segment '%s': %s", segment.c_str(), e.what());
            }
        }
    }

    void publish_callback(const ros::TimerEvent&) {
        joy_andro::JoyAndro msg_to_publish;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            msg_to_publish = current_joy_state_;
        }
        msg_to_publish.header.stamp = ros::Time::now();
        msg_to_publish.header.frame_id = "joyandro_link";
        joy_pub_.publish(msg_to_publish);
    }
    
    void close_client_connection() {
        if (client_sock_ >= 0) {
            close(client_sock_);
            client_sock_ = -1;
            ROS_INFO("Koneksi klien ditutup.");
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
    ros::init(argc, argv, "joyandro_node");
    ros::NodeHandle nh;
    JoyAndroNode node(nh); // Buat objek node
    ros::spin(); // Jalankan ROS event loop
    return 0;
}