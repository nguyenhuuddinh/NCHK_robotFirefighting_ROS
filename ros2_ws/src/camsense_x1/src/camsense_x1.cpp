// Copyright 2020 rossihwang@gmail.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "camsense_x1/camsense_x1.hpp"

#include <algorithm>
#include <cmath>
#include <cinttypes>

#include <rcl_interfaces/msg/parameter.hpp>

class SerialTransportAdapter : public TransportInterface
{
public:
  SerialTransportAdapter()
  : serial_(std::make_shared<serial::Serial>()) {}
  void open() override
  {
    if (serial_->isOpen()) {
      try {serial_->close();} catch (...) {}
    }
    serial_->open();
    failed_ = false;
  }
  void close() override
  {
    failed_ = true;
    serial_->close();
  }
  bool is_open() const override {return !failed_ && serial_->isOpen();}
  size_t available() override {return serial_->available();}
  size_t read(uint8_t * buffer, size_t size) override {return serial_->read(buffer, size);}
  void set_port(const std::string & port) override {serial_->setPort(port);}
  void set_baudrate(uint32_t baudrate) override {serial_->setBaudrate(baudrate);}
  void set_timeout(serial::Timeout timeout) override {serial_->setTimeout(timeout);}
  void recover() override
  {
    if (fatal_error_) {
      throw std::runtime_error("Transport is in a fatal unrecoverable state (close failed)");
    }
    if (serial_) {
      try {
        if (serial_->isOpen()) {
          serial_->close();
        }
      } catch (const std::exception & e) {
        fatal_error_ = true;
        throw std::runtime_error(std::string("Cannot safely recover: ") + e.what());
      } catch (...) {
        fatal_error_ = true;
        throw std::runtime_error("Cannot safely recover: unknown error during close");
      }
    }
    serial_ = std::make_shared<serial::Serial>();
    failed_ = false;
  }

private:
  std::shared_ptr<serial::Serial> serial_;
  bool failed_ = false;
  bool fatal_error_ = false;
};

CamsenseX1::CamsenseX1(
  const std::string & name,
  rclcpp::NodeOptions const & options,
  std::shared_ptr<TransportInterface> transport)
: Node(name, options),
  transport_(transport ? transport : std::make_shared<SerialTransportAdapter>()),
  port_("/dev/ttyUSB0"),
  baud_(115200),
  frame_id_("scan")
{
  create_parameter();

  scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

  // Parser uses external atomic for angle_offset (L3-4: no data race).
  parser_ = std::make_unique<camsense_x1::PacketParser>(
    baud_,
    &angle_offset_,
    [this](const camsense_x1::ScanResult & r) {on_scan(r);});

  // L3-6: Fail-fast on initial serial open failure.
  if (!try_open_serial()) {
    throw std::runtime_error(
            "CamsenseX1: Failed to open serial port " + port_ +
            ". Check hardware connection.");
  }

  // L3-7: Periodic diagnostics independent of successful scans.
  diag_timer_ = this->create_wall_timer(
    std::chrono::seconds(5),
    [this]() {log_diagnostics();});

  thread_ = std::thread{[this]() {rx_loop();}};
}

CamsenseX1::~CamsenseX1()
{
  canceled_.store(true);
  // Wake up any sleeping backoff (L3-6: bounded shutdown).
  wake_cv_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
  if (transport_) {
    if (transport_->is_open()) {
      try {
        transport_->close();
      } catch (...) {
      }
    }
  }
}

bool CamsenseX1::try_open_serial()
{
  try {
    transport_->set_port(port_);
    transport_->set_baudrate(baud_);
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    transport_->set_timeout(to);
    transport_->open();
    if (transport_->is_open()) {
      RCLCPP_INFO(this->get_logger(), "Opened serial port: %s at %d", port_.c_str(), baud_);
      parser_->reset();
      total_reconnect_successes_++;
      return true;
    }
  } catch (const std::exception & e) {
    total_open_errors_++;
    RCLCPP_ERROR(this->get_logger(), "Error opening port %s: %s", port_.c_str(), e.what());
  }
  return false;
}

void CamsenseX1::rx_loop()
{
  bool session_valid = transport_->is_open();

  while (rclcpp::ok() && !canceled_.load()) {
    if (!session_valid || !transport_->is_open()) {
      if (reconnect_attempts_ > 0) {
        int backoff_ms = std::min(
          100 * (1 << std::min(reconnect_attempts_, 6)),
          kMaxReconnectBackoffMs);
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Serial not open. Reconnecting in %d ms (attempt %d)...",
          backoff_ms, reconnect_attempts_ + 1);

        {
          std::unique_lock<std::mutex> lk(wake_mutex_);
          wake_cv_.wait_for(
            lk, std::chrono::milliseconds(backoff_ms),
            [this]() {return canceled_.load();});
        }
        if (canceled_.load()) {break;}
      }

      parser_->reset();
      {
        std::lock_guard<std::mutex> lk(stamp_mutex_);
        have_pending_stamp_ = false;
      }

      try {
        total_reconnect_attempts_++;
        // Attempt recovery to replace broken underlying object
        transport_->recover();

        if (!try_open_serial()) {
          reconnect_attempts_++;
          continue;
        }
        session_valid = true;
      } catch (const std::exception & e) {
        total_open_errors_++;
        RCLCPP_WARN(this->get_logger(), "Reconnect failed: %s", e.what());
        reconnect_attempts_++;
        continue;
      }
    }

    try {
      size_t available = transport_->available();
      if (available > 0) {
        uint8_t buf[512];
        size_t to_read = std::min(available, sizeof(buf));
        size_t read_bytes = transport_->read(buf, to_read);

        // L3-3: Capture ROS time paired with steady-clock time at each read.
        // The parser will record the first-ray steady_ns; on_scan maps it
        // back to the ROS time captured here.
        if (read_bytes > 0) {
          reconnect_attempts_ = 0;  // Session is healthy (L8-3)
          auto steady_now = std::chrono::steady_clock::now();
          int64_t steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            steady_now.time_since_epoch()).count();
          rclcpp::Time ros_now = this->now();
          {
            std::lock_guard<std::mutex> lk(stamp_mutex_);
            first_ray_ros_stamp_ = ros_now;
            first_ray_steady_ns_ = steady_ns;
            have_pending_stamp_ = true;
          }
          parser_->feed(buf, read_bytes, steady_ns);
        }
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    } catch (const std::exception & e) {
      total_read_errors_++;
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Serial read error: %s. Invalidating session.", e.what());

      session_valid = false;
      reconnect_attempts_++;
      try {
        transport_->close();
      } catch (const std::exception & ce) {
        RCLCPP_WARN(this->get_logger(), "Error during transport close: %s", ce.what());
      } catch (...) {
      }
    }
  }
}

void CamsenseX1::on_scan(const camsense_x1::ScanResult & result)
{
  RCLCPP_INFO(this->get_logger(), "on_scan called!");
  sensor_msgs::msg::LaserScan msg;
  msg.header.frame_id = frame_id_;  // frame_id_ is startup-only string.

  // L3-3: Map first-ray steady_ns from ScanResult to ROS time.
  // We use the most recent (ros_time, steady_ns) pair captured at read
  // and offset by the difference to get the ROS time at first-ray.
  {
    std::lock_guard<std::mutex> lk(stamp_mutex_);
    if (have_pending_stamp_ && first_ray_steady_ns_ > 0 &&
      result.first_ray_steady_ns > 0)
    {
      int64_t delta_ns = first_ray_steady_ns_ - result.first_ray_steady_ns;
      // first_ray_ros = latest_ros - delta
      rclcpp::Duration delta(0, 0);
      if (delta_ns >= 0) {
        delta = rclcpp::Duration(
          static_cast<int32_t>(delta_ns / 1000000000LL),
          static_cast<uint32_t>(delta_ns % 1000000000LL));
        msg.header.stamp = first_ray_ros_stamp_ - delta;
      } else {
        int64_t abs_ns = -delta_ns;
        delta = rclcpp::Duration(
          static_cast<int32_t>(abs_ns / 1000000000LL),
          static_cast<uint32_t>(abs_ns % 1000000000LL));
        msg.header.stamp = first_ray_ros_stamp_ + delta;
      }
    } else {
      msg.header.stamp = this->now();
    }
  }

  msg.angle_min = 0.0;
  msg.angle_max = -2.0 * M_PI + (2.0 * M_PI / 400.0);
  msg.angle_increment = -(2.0 * M_PI) / 400.0;
  msg.scan_time = static_cast<float>(result.scan_duration_s);
  msg.time_increment = 0.0;
  msg.range_min = static_cast<float>(camsense_x1::kRangeMinM);
  msg.range_max = static_cast<float>(camsense_x1::kRangeMaxM);

  msg.ranges.assign(
    result.ranges,
    result.ranges + camsense_x1::kBinsPerRevolution);
  msg.intensities.assign(
    result.intensities,
    result.intensities + camsense_x1::kBinsPerRevolution);

  scan_pub_->publish(msg);
  int64_t steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  last_publish_steady_ns_.store(steady_ns, std::memory_order_relaxed);
}

// L3-7: Periodic diagnostics independent of scan callback.
void CamsenseX1::log_diagnostics()
{
  if (!parser_) {return;}
  auto c = parser_->counters();
  auto now_tp = std::chrono::steady_clock::now();
  int age_ms = -1;
  int64_t last_pub_ns = last_publish_steady_ns_.load(std::memory_order_relaxed);
  if (last_pub_ns > 0) {
    int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      now_tp.time_since_epoch()).count();
    age_ms = static_cast<int>((now_ns - last_pub_ns) / 1000000LL);
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Diag: pub=%" PRIu64 " drop=%" PRIu64 " (stale=%" PRIu64 ") pkt=%" PRIu64 " rej=%" PRIu64
    " disc=%" PRIu64 " dup=%" PRIu64 " ooo=%" PRIu64 " crst=%" PRIu64 " gap=%" PRIu64
    " chk=%" PRIu64 " spd_avg=%.0f age=%dms rd_err=%" PRIu64 " opn_err=%" PRIu64
    " recon=%" PRIu64 "/%" PRIu64 " t_fb(proc=%" PRIu64 ",cache=%" PRIu64 ",nom=%" PRIu64 ")",
    static_cast<uint64_t>(c.revolutions_published),
    static_cast<uint64_t>(c.revolutions_dropped),
    static_cast<uint64_t>(c.stale_revolutions_dropped),
    static_cast<uint64_t>(c.packets_accepted),
    static_cast<uint64_t>(c.candidates_rejected),
    static_cast<uint64_t>(c.bytes_discarded),
    static_cast<uint64_t>(c.duplicates_detected),
    static_cast<uint64_t>(c.out_of_order_detected),
    static_cast<uint64_t>(c.continuity_resets),
    static_cast<uint64_t>(c.gap_resets),
    static_cast<uint64_t>(c.checksum_failures),
    c.speed_count > 0 ? static_cast<double>(c.speed_sum) / c.speed_count : 0.0,
    age_ms,
    static_cast<uint64_t>(total_read_errors_.load(std::memory_order_relaxed)),
    static_cast<uint64_t>(total_open_errors_.load(std::memory_order_relaxed)),
    static_cast<uint64_t>(total_reconnect_successes_.load(std::memory_order_relaxed)),
    static_cast<uint64_t>(total_reconnect_attempts_.load(std::memory_order_relaxed)),
    static_cast<uint64_t>(c.proc_clock_fallbacks),
    static_cast<uint64_t>(c.cache_fallbacks),
    static_cast<uint64_t>(c.nominal_fallbacks));
}

void CamsenseX1::create_parameter()
{
  frame_id_ = declare_parameter<std::string>("frame_id", "scan");
  port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
  baud_ = declare_parameter<int>("baud", 115200);
  angle_offset_.store(declare_parameter<int>("angle_offset", 0));

  param_sub_ = this->add_on_set_parameters_callback(
    [this](std::vector<rclcpp::Parameter> parameters)
    -> rcl_interfaces::msg::SetParametersResult {
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;
      for (auto const & p : parameters) {
        if (!handle_parameter(p)) {
          result.successful = false;
        }
      }
      return result;
    });
}

bool CamsenseX1::handle_parameter(rclcpp::Parameter const & param)
{
  if (param.get_name() == "frame_id") {
    // frame_id is startup-only; reject runtime change.
    RCLCPP_ERROR(
      this->get_logger(),
      "Dynamic change of frame_id not supported. Restart required.");
    return false;
  } else if (param.get_name() == "port") {
    RCLCPP_ERROR(
      this->get_logger(),
      "Dynamic change of port not supported. Restart required.");
    return false;
  } else if (param.get_name() == "baud") {
    RCLCPP_ERROR(
      this->get_logger(),
      "Dynamic change of baud not supported. Restart required.");
    return false;
  } else if (param.get_name() == "angle_offset") {
    // L3-4: atomic update, no mutex needed. Safe for concurrent read.
    angle_offset_.store(
      static_cast<int>(param.as_int()) % 360,
      std::memory_order_relaxed);
  } else {
    return false;
  }
  return true;
}
