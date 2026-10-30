#include "observer.hpp"

#include <algorithm>
#include <cctype>
#include <string>

#include <QBuffer>
#include <QImageReader>

Observer::Observer(bool preview, bool mapping) : preview_(preview), mapping_(mapping)
{
  node_ = std::make_shared<rclcpp::Node>("fire_robot_ui_observer");
  const auto qos = rclcpp::SensorDataQoS().keep_last(1);
  imageSub_ = node_->create_subscription<sensor_msgs::msg::CompressedImage>(
    mapping_ ? "/image_raw/compressed" : "/yolo/image_annotated/compressed", qos,
    [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg, const rclcpp::MessageInfo & info) {
      imageCallback(msg, info);
    });
  operatorSub_ = node_->create_subscription<std_msgs::msg::String>(
    "/fire_mission/operator_state", qos,
    [this](std_msgs::msg::String::ConstSharedPtr msg, const rclcpp::MessageInfo & info) {
      textCallback(msg, false, info);
    });
  actuatorSub_ = node_->create_subscription<std_msgs::msg::String>(
    "/fire_sensors/actuator_state", qos,
    [this](std_msgs::msg::String::ConstSharedPtr msg, const rclcpp::MessageInfo & info) {
      textCallback(msg, true, info);
    });
  executor_.add_node(node_);
  thread_ = std::thread([this]() {
      try {
        executor_.spin();
      } catch (const std::exception & exc) {
        RCLCPP_ERROR(node_->get_logger(), "Observer stopped: %s", exc.what());
      }
    });
}

Observer::~Observer()
{
  executor_.cancel();
  if (thread_.joinable()) {
    thread_.join();
  }
  executor_.remove_node(node_);
}

Observer::Snapshot Observer::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_;
}

bool Observer::sourceMatches(const std::string & topic, const std::string & expectedNode,
  const std::string & type, const rclcpp::MessageInfo & info, size_t pin)
{
  try {
    const auto endpoints = node_->get_publishers_info_by_topic(topic);
    if (endpoints.size() != 1 || endpoints[0].node_name() != expectedNode ||
      endpoints[0].node_namespace() != "/" || endpoints[0].topic_type() != type ||
      endpoints[0].qos_profile().get_rmw_qos_profile().reliability != RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT) {
      return false;
    }
    const auto & gid = endpoints[0].endpoint_gid();
    const auto & received = info.get_rmw_message_info().publisher_gid;
    if (std::none_of(gid.begin(), gid.end(), [](uint8_t x) {return x != 0;}) ||
      !std::equal(gid.begin(), gid.end(), received.data) ||
      (pinned_[pin] && pins_[pin] != gid)) {
      return false;
    }
    pins_[pin] = gid;
    pinned_[pin] = true;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

void Observer::imageCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg,
  const rclcpp::MessageInfo & info)
{
  const auto received = Clock::now();
  if (!sourceMatches(mapping_ ? "/image_raw/compressed" : "/yolo/image_annotated/compressed",
      preview_ ? "fire_robot_control_preview" : (mapping_ ? "native_mjpeg_camera" : "yolo_observer"),
      "sensor_msgs/msg/CompressedImage", info, 0)) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_.imageValid = false;
    return;
  }
  const auto stampNs = static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
    msg->header.stamp.nanosec;
  const double ageMs = (node_->get_clock()->now().nanoseconds() - stampNs) / 1000000.0;
  std::string format = msg->format;
  std::transform(format.begin(), format.end(), format.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  const bool validHeader = stampNs > lastImageStampNs_ && ageMs >= -20 && ageMs <= 300 &&
    (msg->header.frame_id == "camera_frame" || msg->header.frame_id == "camera_optical_frame") &&
    (format == "jpeg" || format == "jpg") && !msg->data.empty() && msg->data.size() <= 2000000;
  QImage decoded;
  if (validHeader) {
    const QByteArray bytes(reinterpret_cast<const char *>(msg->data.data()),
      static_cast<int>(msg->data.size()));
    QBuffer buffer;
    buffer.setData(bytes);
    buffer.open(QIODevice::ReadOnly);
    QImageReader reader(&buffer, "JPEG");
    const QSize size = reader.size();
    if (size.width() > 0 && size.height() > 0 &&
      static_cast<qint64>(size.width()) * size.height() <= 8000000) {
      decoded = reader.read();
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);
  latest_.imageValid = validHeader && !decoded.isNull();
  latest_.imageReceived = received;
  latest_.imageSourceAgeMs = std::max(0.0, ageMs);
  if (latest_.imageValid) {
    lastImageStampNs_ = stampNs;
    latest_.image = std::move(decoded);
  }
}

void Observer::textCallback(const std_msgs::msg::String::ConstSharedPtr & msg, bool actuator,
  const rclcpp::MessageInfo & info)
{
  const auto received = Clock::now();
  const bool source = sourceMatches(actuator ? "/fire_sensors/actuator_state" :
    "/fire_mission/operator_state", preview_ ? "fire_robot_control_preview" :
    (actuator ? "serial_bridge_node" : "fire_robot_operator"), "std_msgs/msg/String", info,
    actuator ? 2 : 1);
  QString text;
  if (source && msg->data.size() <= 256 * 1024) {
    const QByteArray raw(msg->data.data(), static_cast<int>(msg->data.size()));
    const QString decoded = QString::fromUtf8(raw);
    if (decoded.toUtf8() == raw) {text = decoded;}
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (actuator) {
    latest_.actuatorJson = text;
    latest_.actuatorReceived = received;
  } else {
    latest_.operatorJson = text;
    latest_.operatorReceived = received;
  }
}
