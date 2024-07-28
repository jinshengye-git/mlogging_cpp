#pragma once

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <map>
#include <mutex>
#include <atomic>
#include <boost/function.hpp>
#include <netinet/in.h>

class UDPServer {
private:
    int socketFd;
    struct sockaddr_in serverAddr, clientAddr;
    char* buffer;
    std::ofstream logFile;
    ros::NodeHandle nh;
    std::map<std::string, ros::Subscriber> subscribers;
    std::mutex mutex;
    std::atomic<bool> running;

    std::string getCurrentTime() const;
    void logMessage(const std::string& topic, const std::string& message);
    
    template<typename T>
    void topicCallback(const typename T::ConstPtr& msg, const std::string& topic) {
        std::stringstream ss;
        ss << *msg;
        logMessage(topic, ss.str());
    }

    void runUDPServer();

public:
    UDPServer(int port, int bufferSize);
    ~UDPServer();

    template<typename T>
    void subscribeToTopic(const std::string& topic) {
        subscribers[topic] = nh.subscribe<T>(topic, 1000, 
            boost::bind(&UDPServer::topicCallback<T>, this, _1, topic));
        std::cout << "Subscribed to ROS topic: " << topic << std::endl;
    }

    void run();
    void stop();
};