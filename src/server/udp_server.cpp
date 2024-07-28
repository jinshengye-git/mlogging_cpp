#include "server/udp_server.hpp"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>

UDPServer::UDPServer(int port, int bufferSize) : buffer(new char[bufferSize]), running(true) {
    socketFd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socketFd < 0) {
        std::cerr << "Error creating socket" << std::endl;
        exit(EXIT_FAILURE);
    }

    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(port);

    if (bind(socketFd, (const struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0) {
        std::cerr << "Bind failed" << std::endl;
        exit(EXIT_FAILURE);
    }

    logFile.open("ros_topics.log", std::ios::app);
    if (!logFile.is_open()) {
        std::cerr << "Failed to open log file" << std::endl;
        exit(EXIT_FAILURE);
    }
}

UDPServer::~UDPServer() {
    close(socketFd);
    logFile.close();
    delete[] buffer;
}

std::string UDPServer::getCurrentTime() const {
    auto now = std::chrono::system_clock::now();
    auto inTimeT = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&inTimeT), "%Y-%m-%d %X");
    return ss.str();
}

void UDPServer::logMessage(const std::string& topic, const std::string& message) {
    std::lock_guard<std::mutex> lock(mutex);
    logFile << "[" << getCurrentTime() << "] " << topic << ": " << message << std::endl;
    logFile.flush();
    std::cout << "Logged: " << topic << " - " << message << std::endl;
}

void UDPServer::runUDPServer() {
    std::cout << "UDP Server is running on port " << ntohs(serverAddr.sin_port) << std::endl;

    while (running && ros::ok()) {
        fd_set readSet;
        FD_ZERO(&readSet);
        FD_SET(socketFd, &readSet);

        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        int result = select(socketFd + 1, &readSet, NULL, NULL, &timeout);

        if (result > 0) {
            socklen_t len = sizeof(clientAddr);
            int n = recvfrom(socketFd, buffer, 1024, MSG_WAITALL, 
                             (struct sockaddr *)&clientAddr, &len);
            if (n > 0) {
                buffer[n] = '\0';

                std::string receivedData(buffer);
                size_t delimiterPos = receivedData.find(':');
                if (delimiterPos != std::string::npos) {
                    std::string topic = receivedData.substr(0, delimiterPos);
                    std::string message = receivedData.substr(delimiterPos + 1);
                    
                    logMessage(topic, message);
                } else {
                    std::cout << "Received invalid format from UDP: " << receivedData << std::endl;
                }
            }
        }
    }
   
}

void UDPServer::run() {
    std::thread udpThread(&UDPServer::runUDPServer, this);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    udpThread.join();
}

void UDPServer::stop() {
    running = false;
}