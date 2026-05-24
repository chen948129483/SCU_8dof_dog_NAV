#include "LIVMapper.h"
#include <signal.h>
#include <iostream>

// Global pointer to mapper for signal handler
LIVMapper* g_mapper = nullptr;

// Signal handler to save PCD before exiting
void signal_handler(int signum) {
  std::cout << "\n[INFO] Caught signal " << signum << ", saving PCD before exit..." << std::endl;
  if (g_mapper != nullptr) {
    g_mapper->savePCD();
    std::cout << "[INFO] PCD saved successfully" << std::endl;
  }
  exit(signum);
}

int main(int argc, char **argv)
{
  // Register signal handlers
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  rclcpp::Node::SharedPtr nh;
  image_transport::ImageTransport it_(nh);
  LIVMapper mapper(nh, "laserMapping", options);
  g_mapper = &mapper;  // Set global pointer for signal handler
  mapper.initializeSubscribersAndPublishers(nh, it_);
  mapper.run(nh);
  
  // Save PCD on normal exit
  mapper.savePCD();
  
  rclcpp::shutdown();
  return 0;
}
