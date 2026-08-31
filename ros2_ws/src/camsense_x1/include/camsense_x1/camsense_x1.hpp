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

#ifndef CAMSENSE_X1__CAMSENSE_X1_HPP_
#define CAMSENSE_X1__CAMSENSE_X1_HPP_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "serial/serial.h"

#include "camsense_x1/camsense_x1_parser.hpp"

class TransportInterface
{
public:
  virtual ~TransportInterface() = default;
  virtual void open() = 0;
  virtual void close() = 0;
  virtual bool is_open() const = 0;
  virtual size_t available() = 0;
  virtual size_t read(uint8_t * buffer, size_t size) = 0;
  virtual void set_port(const std::string & port) = 0;
  virtual void set_baudrate(uint32_t baudrate) = 0;
  virtual void set_timeout(serial::Timeout timeout) = 0;
  virtual void recover() = 0;
};

/// \brief ROS 2 node adapter: serial I/O -> PacketParser -> LaserScan publisher.
class CamsenseX1 : public rclcpp::Node
{
public:
  CamsenseX1(
    const std::string & name, rclcpp::NodeOptions const & options,
    std::shared_ptr<TransportInterface> transport = nullptr);
  ~CamsenseX1();

private:
  // ROS
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::TimerBase::SharedPtr diag_timer_;

  // Serial/Transport
  std::shared_ptr<TransportInterface> transport_;
  std::string port_;
  int baud_;

  // Parameters — startup-only for port/baud, atomic for angle_offset (L3-4).
  std::string frame_id_;
  std::atomic<int> angle_offset_{0};

  // Parser (used only from RX thread).
  std::unique_ptr<camsense_x1::PacketParser> parser_;

  // RX thread + shutdown
  std::thread thread_;
  std::atomic<bool> canceled_{false};
  std::mutex wake_mutex_;
  std::condition_variable wake_cv_;

  // Reconnect and node diagnostics
  int reconnect_attempts_ = 0;  // Local to rx_loop
  static constexpr int kMaxReconnectBackoffMs = 5000;
  std::atomic<uint64_t> total_reconnect_attempts_{0};
  std::atomic<uint64_t> total_reconnect_successes_{0};
  std::atomic<uint64_t> total_read_errors_{0};
  std::atomic<uint64_t> total_open_errors_{0};
  std::atomic<int64_t> last_publish_steady_ns_{0};

  // Silence timeout guard
  std::atomic<uint64_t> total_silence_timeouts_{0};
  int64_t last_rx_steady_ns_ = 0;
  // 5.2Hz implies ~192ms/rev. 2000ms is >10 revolutions, generous but fast enough to recover.
  static constexpr int64_t kTransportSilenceTimeoutNs = 2000000000LL;

  // First-ray ROS timestamp tracking (L3-3).
  // Stored per-revolution; set when parser starts a new revolution.
  std::mutex stamp_mutex_;
  rclcpp::Time first_ray_ros_stamp_;
  int64_t first_ray_steady_ns_ = 0;
  bool have_pending_stamp_ = false;

  // Methods
  void rx_loop();
  bool try_open_serial();
  void on_scan(const camsense_x1::ScanResult & result);
  void log_diagnostics();
  void create_parameter();
  bool handle_parameter(rclcpp::Parameter const & param);

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_sub_;
};

#endif  // CAMSENSE_X1__CAMSENSE_X1_HPP_
