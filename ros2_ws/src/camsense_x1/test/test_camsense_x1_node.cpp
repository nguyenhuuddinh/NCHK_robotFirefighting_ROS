// Copyright 2024 rossihwang@gmail.com
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

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "camsense_x1/camsense_x1.hpp"
#include "camsense_helpers.hpp"

using namespace std::chrono_literals;

class FakeTransport : public TransportInterface
{
public:
  explicit FakeTransport(bool bad_close = false, int failures = 0, bool initial_failure = false)
  : bad_close_(bad_close), failures_(failures), initial_failure_(initial_failure) {}

  void open() override
  {
    const int n = ++opens;
    if (initial_failure_ || (n > 1 && n <= 1 + failures_)) {
      throw std::runtime_error("fake open failure");
    }
    opened = true;
  }
  void close() override
  {
    ++closes;
    if (bad_close_) {throw std::runtime_error("fake close failure; is_open remains true");}
    opened = false;
  }
  bool is_open() const override {return opened.load();}
  size_t available() override
  {
    if (armed.load()) {
      ++read_errors;
      std::this_thread::sleep_for(1ms);
      throw std::runtime_error("fake available error");
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    return rx_data_.size();
  }
  size_t read(uint8_t * buf, size_t len) override
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    size_t count = std::min(len, rx_data_.size());
    if (count > 0) {
      std::memcpy(buf, rx_data_.data(), count);
      rx_data_.erase(rx_data_.begin(), rx_data_.begin() + count);
    }
    return count;
  }

  void inject(const std::vector<uint8_t> & data)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    rx_data_.insert(rx_data_.end(), data.begin(), data.end());
  }

  void set_port(const std::string &) override {}
  void set_baudrate(uint32_t) override {}
  void set_timeout(serial::Timeout) override {}
  void recover() override
  {
    if (fatal_error_) {
      throw std::runtime_error("fatal");
    }
    try {
      close();
    } catch (...) {
      fatal_error_ = true;
      throw std::runtime_error("fatal");
    }
    opened = false;
  }

  std::atomic<bool> opened{false}, armed{false};
  std::atomic<int> opens{0}, closes{0}, read_errors{0};

private:
  bool bad_close_;
  int failures_;
  bool initial_failure_;
  std::vector<uint8_t> rx_data_;
  std::mutex data_mutex_;
  bool fatal_error_{false};
};

bool wait_until(std::function<bool()> ready, int timeout_ms = 1500)
{
  const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < end) {
    if (ready()) {return true;}
    std::this_thread::sleep_for(2ms);
  }
  return ready();
}

TEST(NodeTest, InitialFailFast)
{
  rclcpp::NodeOptions options;
  bool initial_ok = false;
  try {
    auto fake = std::make_shared<FakeTransport>(false, 0, true);
    auto node = std::make_shared<CamsenseX1>("fake_initial", options, fake);
  } catch (const std::runtime_error &) {
    initial_ok = true;
  }
  EXPECT_TRUE(initial_ok);
}

TEST(NodeTest, SingleFaultRecovery)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("frame_id", "laser_frame")});
  auto fake = std::make_shared<FakeTransport>();
  auto node = std::make_shared<CamsenseX1>("fake_normal", options, fake);

  sensor_msgs::msg::LaserScan::SharedPtr received_scan;
  std::mutex scan_mutex;
  auto sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10, [&](sensor_msgs::msg::LaserScan::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(scan_mutex);
      received_scan = msg;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spin_thread = std::thread([&executor]() {executor.spin();});

  // Inject a single fault
  fake->armed = true;
  bool fault_injected = wait_until([&]() {return fake->read_errors >= 1;});
  EXPECT_TRUE(fault_injected);

  // Disarm and expect recovery
  fake->armed = false;
  bool recovered = wait_until([&]() {return fake->opens >= 2 && fake->opened;});
  EXPECT_TRUE(recovered);

  // Feed synthetic data in a loop until received
  std::vector<uint8_t> rev_data;
  auto rev_pkts = make_full_revolution();
  for (const auto & pkt : rev_pkts) {
    rev_data.insert(rev_data.end(), pkt.begin(), pkt.end());
  }
  // Wrap packet
  auto wrap_pkt = make_packet(0.0, 6.8);
  rev_data.insert(rev_data.end(), wrap_pkt.begin(), wrap_pkt.end());

  auto next_inject = std::chrono::steady_clock::now();
  bool msg_received = wait_until(
    [&]() {
      if (std::chrono::steady_clock::now() >= next_inject) {
        fake->inject(rev_data);
        next_inject = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
      }
      std::lock_guard<std::mutex> lock(scan_mutex);
      return received_scan != nullptr;
    }, 3000);

  EXPECT_TRUE(msg_received);
  if (msg_received) {
    std::lock_guard<std::mutex> lock(scan_mutex);
    EXPECT_EQ(received_scan->header.frame_id, "laser_frame");
  }

  executor.cancel();
  spin_thread.join();
}

TEST(NodeTest, PersistentFaultBoundedRetry)
{
  rclcpp::NodeOptions options;
  auto fake = std::make_shared<FakeTransport>(false, 0);  // No open failures, just read failures
  auto node = std::make_shared<CamsenseX1>("fake_persistent", options, fake);

  fake->armed = true;
  // Wait a bit to let it retry
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // The backoff should limit the number of read errors
  // Initially 0, then backoff increases: 200, 400. In 500ms we expect at most ~3-4 errors.
  EXPECT_GT(fake->read_errors, 0);
  EXPECT_LT(fake->read_errors, 10);  // Not hot-looping!
}

TEST(NodeTest, ShutdownBackoff)
{
  rclcpp::NodeOptions options;
  auto fake = std::make_shared<FakeTransport>(false, 1000);
  auto node = std::make_shared<CamsenseX1>("fake_cancel", options, fake);
  fake->armed = true;
  bool entered = wait_until([&]() {return fake->opens >= 2;});
  EXPECT_TRUE(entered);
  std::this_thread::sleep_for(20ms);
  const auto begin = std::chrono::steady_clock::now();
  node.reset();
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - begin).count();
  EXPECT_LT(ms, 500);
}

TEST(NodeTest, CloseFailureFatal)
{
  rclcpp::NodeOptions options;
  auto fake = std::make_shared<FakeTransport>(true);  // Bad close
  auto node = std::make_shared<CamsenseX1>("fake_close_error", options, fake);
  fake->armed = true;

  bool fault_injected = wait_until([&]() {return fake->read_errors >= 1;});
  EXPECT_TRUE(fault_injected);

  bool attempted_close = wait_until([&]() {return fake->closes >= 1;});
  EXPECT_TRUE(attempted_close);

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_EQ(fake->opens.load(), 1);  // No new opens, object is fatal!
}

TEST(NodeTest, LaserScanPublishing)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    rclcpp::Parameter("frame_id", "laser_frame")
  });
  auto fake = std::make_shared<FakeTransport>();
  auto node = std::make_shared<CamsenseX1>("fake_publish", options, fake);

  sensor_msgs::msg::LaserScan::SharedPtr received_scan;
  std::mutex scan_mutex;
  auto sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10, [&](sensor_msgs::msg::LaserScan::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(scan_mutex);
      received_scan = msg;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spin_thread = std::thread([&executor]() {executor.spin();});

  // Wait for matching
  wait_until(
    [&]() {
      return sub->get_publisher_count() > 0;
    }, 1000);

  // Inject a synthetic valid revolution
  std::vector<uint8_t> rev_data;
  auto rev_pkts = make_full_revolution();
  for (const auto & pkt : rev_pkts) {
    rev_data.insert(rev_data.end(), pkt.begin(), pkt.end());
  }
  auto wrap_pkt = make_packet(0.0, 6.8);
  rev_data.insert(rev_data.end(), wrap_pkt.begin(), wrap_pkt.end());

  auto next_inject = std::chrono::steady_clock::now();
  bool msg_received = wait_until(
    [&]() {
      if (std::chrono::steady_clock::now() >= next_inject) {
        fake->inject(rev_data);
        next_inject = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
      }
      std::lock_guard<std::mutex> lock(scan_mutex);
      return received_scan != nullptr;
    }, 3000);
  EXPECT_TRUE(msg_received);
  if (msg_received) {
    std::lock_guard<std::mutex> lock(scan_mutex);
    EXPECT_EQ(received_scan->header.frame_id, "laser_frame");
    EXPECT_EQ(received_scan->ranges.size(), 400u);
    EXPECT_NEAR(received_scan->time_increment, 0.0, 1e-6);
  }

  executor.cancel();
  spin_thread.join();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
