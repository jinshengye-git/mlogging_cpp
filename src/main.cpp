#include "server/udp_server.hpp"
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <roboline/status.h>
#include <roboline/pos.h>
#include <signal.h>
#include <memory>

int PORT = 37171;
int BUFFER_SIZE = 1024;
std::unique_ptr<UDPServer> server;
std::atomic<bool> shouldRestart(false);

void signalHandler(int signum) {
    switch (signum) {
        case SIGINT:
            std::cout << "Interrupt signal (SIGINT) received. Shutting down...\n";
            break;
        case SIGHUP:
            std::cout << "Hangup signal (SIGHUP) received. Restarting...\n";
            shouldRestart = true;
            break;
        case SIGTERM:
            std::cout << "Termination signal (SIGTERM) received. Shutting down...\n";
            break;
        default:
            std::cout << "Unexpected signal (" << signum << ") received. Shutting down...\n";
    }

    if (server) {
        server->stop();
    }
    ros::shutdown();
}

void runServer() {
    server = std::make_unique<UDPServer>(PORT, BUFFER_SIZE);
    server->subscribeToTopic<std_msgs::String>("/topic1");
    server->subscribeToTopic<sensor_msgs::Image>("/camera/image_raw");
    server->subscribeToTopic<roboline::pos>("/direction");
    server->subscribeToTopic<nav_msgs::Odometry>("/odom");
    server->run();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "udp_server_node", ros::init_options::NoSigintHandler);

    signal(SIGINT, signalHandler);
    signal(SIGHUP, signalHandler);
    signal(SIGTERM, signalHandler);

    do {
        shouldRestart = false;
        ros::init(argc, argv, "udp_server_node", ros::init_options::NoSigintHandler);
        runServer();

        if (shouldRestart) {
            std::cout << "Restarting server...\n";
            // Clean up before restarting
            server.reset();
            ros::shutdown();
        }
    } while (shouldRestart);

    return 0;
}