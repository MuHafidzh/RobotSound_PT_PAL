#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <map>
#include <iomanip>
#include <cmath>
#include <unistd.h>
#include <csignal>
#include <cstring>
#include <cerrno>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

class BluetoothReceiver {
private:
    int server_sock = -1;
    int client_sock = -1;
    bool running = true;

    // Variabel untuk menyimpan state terakhir agar tidak spam di console
    std::map<std::string, bool> lastButtonStates;
    float lastJoystickX = -999.0f;
    float lastJoystickY = -999.0f;

public:
    // Inisialisasi soket server Bluetooth
    bool initialize() {
        server_sock = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
        if (server_sock < 0) {
            std::cerr << "Gagal membuat soket: " << strerror(errno) << std::endl;
            return false;
        }

        struct sockaddr_rc loc_addr = {0};
        loc_addr.rc_family = AF_BLUETOOTH;
        loc_addr.rc_bdaddr = {{0, 0, 0, 0, 0, 0}}; // BDADDR_ANY
        loc_addr.rc_channel = (uint8_t)1;

        if (bind(server_sock, (struct sockaddr *)&loc_addr, sizeof(loc_addr)) < 0) {
            std::cerr << "Gagal melakukan bind soket: " << strerror(errno) << std::endl;
            close(server_sock);
            return false;
        }

        if (listen(server_sock, 1) < 0) {
            std::cerr << "Gagal listen pada soket: " << strerror(errno) << std::endl;
            close(server_sock);
            return false;
        }

        std::cout << "Server Bluetooth listening di channel 1..." << std::endl;
        return true;
    }

    // Loop utama untuk menerima koneksi dan data
    void run() {
        while (running) {
            waitForConnection();
            if (client_sock >= 0) {
                startListening();
                closeClientConnection();
            }
        }
    }

    // Berhenti saat menerima sinyal (misal: Ctrl+C)
    void stop() {
        running = false;
        closeClientConnection();
        if (server_sock >= 0) {
            close(server_sock);
            server_sock = -1;
        }
    }

    ~BluetoothReceiver() {
        stop();
    }

private:
    // Menunggu koneksi dari perangkat Android
    void waitForConnection() {
        std::cout << "\nMenunggu koneksi dari controller..." << std::endl;
        struct sockaddr_rc rem_addr = {0};
        socklen_t opt = sizeof(rem_addr);
        client_sock = accept(server_sock, (struct sockaddr *)&rem_addr, &opt);

        if (client_sock < 0) {
            std::cerr << "Gagal menerima koneksi: " << strerror(errno) << std::endl;
            return;
        }

        char addr_str[19] = {0};
        ba2str(&rem_addr.rc_bdaddr, addr_str);
        std::cout << "Koneksi diterima dari " << addr_str << std::endl;
        std::cout << "========================================" << std::endl;
    }

    // Loop untuk membaca data dari soket klien
    void startListening() {
        char buffer[1024] = {0};
        std::string partial_data; // Buffer untuk menangani data yang terpotong

        while (running) {
            memset(buffer, 0, sizeof(buffer));
            int bytes_read = read(client_sock, buffer, sizeof(buffer) - 1);

            if (bytes_read > 0) {
                partial_data += buffer;
                size_t newline_pos;
                
                // Proses semua frame lengkap yang diakhiri dengan newline
                while ((newline_pos = partial_data.find('\n')) != std::string::npos) {
                    std::string frame = partial_data.substr(0, newline_pos);
                    partial_data.erase(0, newline_pos + 1);

                    if (!frame.empty()) {
                        parseConsolidatedData(frame);
                    }
                }
            } else {
                if (bytes_read == 0) {
                    std::cout << "Klien terputus." << std::endl;
                } else {
                    std::cerr << "Read error: " << strerror(errno) << std::endl;
                }
                break;
            }
        }
    }

    // Parsing frame data terpadu (misal: "joy:0.5,0.2;i:1;b:0;m:0;a:0")
    void parseConsolidatedData(const std::string& data) {
        std::istringstream dataStream(data);
        std::string segment;

        // Pisahkan frame berdasarkan ';'
        while (std::getline(dataStream, segment, ';')) {
            size_t colonPos = segment.find(':');
            if (colonPos == std::string::npos) continue;

            std::string key = segment.substr(0, colonPos);
            std::string value = segment.substr(colonPos + 1);

            if (key == "joy") {
                parseJoystickData(value);
            } else {
                parseButtonData(key, value);
            }
        }
    }

    void parseJoystickData(const std::string& value) {
        std::istringstream valueStream(value);
        std::string x_str, y_str;
        if (std::getline(valueStream, x_str, ',') && std::getline(valueStream, y_str, ',')) {
            try {
                float x = std::stof(x_str);
                float y = std::stof(y_str);
                processJoystickInput(x, y);
            } catch (const std::exception& e) {
                // Abaikan jika ada error parsing
            }
        }
    }

    void parseButtonData(const std::string& key, const std::string& value) {
        try {
            bool isPressed = (std::stoi(value) == 1);
            processButtonInput(key, isPressed);
        } catch (const std::exception& e) {
            // Abaikan jika ada error parsing
        }
    }

    // Menampilkan data joystick jika nilainya berubah
    void processJoystickInput(float x, float y) {
        if (std::abs(x - lastJoystickX) > 0.001f || std::abs(y - lastJoystickY) > 0.001f) {
            std::cout << "Joystick - X: " << std::fixed << std::setprecision(3) << x 
                      << ", Y: " << std::fixed << std::setprecision(3) << y << std::endl;
            lastJoystickX = x;
            lastJoystickY = y;
        }
    }

    // Menampilkan status tombol jika statusnya berubah
    void processButtonInput(const std::string& buttonKey, bool isPressed) {
        if (lastButtonStates.find(buttonKey) == lastButtonStates.end() || lastButtonStates[buttonKey] != isPressed) {
            std::string buttonName = "Unknown Button";
            if (buttonKey == "i") buttonName = "IDLE (Triangle)";
            else if (buttonKey == "b") buttonName = "BOOST (Square)";
            else if (buttonKey == "m") buttonName = "MANUAL (Circle)";
            else if (buttonKey == "a") buttonName = "AUTO (X)";

            std::cout << "Button " << buttonName << " -> " << (isPressed ? "PRESSED" : "RELEASED") << std::endl;
            lastButtonStates[buttonKey] = isPressed;
        }
    }

    void closeClientConnection() {
        if (client_sock >= 0) {
            close(client_sock);
            client_sock = -1;
            // Reset status tombol untuk koneksi berikutnya
            lastButtonStates.clear();
            lastJoystickX = -999.0f;
            lastJoystickY = -999.0f;
        }
    }
};

// Global receiver untuk signal handling
BluetoothReceiver* global_receiver_ptr = nullptr;

void signalHandler(int signum) {
    std::cout << "\nSinyal diterima, mematikan server..." << std::endl;
    if (global_receiver_ptr) {
        global_receiver_ptr->stop();
    }
    exit(signum);
}

int main() {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    BluetoothReceiver receiver;
    global_receiver_ptr = &receiver;

    if (!receiver.initialize()) {
        return -1;
    }

    std::cout << "========================================" << std::endl;
    std::cout << "  Bluetooth Joystick Receiver" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Tekan Ctrl+C untuk keluar." << std::endl;

    receiver.run();

    return 0;
}