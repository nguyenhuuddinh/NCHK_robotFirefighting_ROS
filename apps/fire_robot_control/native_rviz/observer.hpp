#pragma once

#include <atomic>
#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

#include <QImage>
#include <QString>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>

// Read-only observer: it never publishes ROS topics or creates action clients.
class Observer
{
public:
  using Clock = std::chrono::steady_clock;
  struct Snapshot
  {
    QImage image;
    bool imageValid = false;
    Clock::time_point imageReceived{};
    double imageSourceAgeMs = 0;
    QString operatorJson;
    Clock::time_point operatorReceived{};
    QString actuatorJson;
    Clock::time_point actuatorReceived{};
  };

  explicit Observer(bool preview, bool mapping = false);
  ~Observer();
  Observer(const Observer &) = delete;
  Observer & operator=(const Observer &) = delete;
  Snapshot snapshot() const;

private:
  void imageCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg,
    const rclcpp::MessageInfo & info);
  void textCallback(const std_msgs::msg::String::ConstSharedPtr & msg, bool actuator,
    const rclcpp::MessageInfo & info);
  bool sourceMatches(const std::string & topic, const std::string & expectedNode,
    const std::string & type, const rclcpp::MessageInfo & info, size_t pin);

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr imageSub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr operatorSub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr actuatorSub_;
  mutable std::mutex mutex_;
  Snapshot latest_;
  bool preview_;
  bool mapping_;
  std::array<std::array<uint8_t, RMW_GID_STORAGE_SIZE>, 3> pins_{};
  std::array<bool, 3> pinned_{};
  int64_t lastImageStampNs_ = 0;
  std::thread thread_;
};
