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

/// \file
/// \brief Pure-C++ byte-stream parser and revolution assembler for Camsense X1.
///        No ROS or serial dependency. Designed for deterministic unit testing.

#ifndef CAMSENSE_X1__CAMSENSE_X1_PARSER_HPP_
#define CAMSENSE_X1__CAMSENSE_X1_PARSER_HPP_

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <utility>

namespace camsense_x1
{

// ---------------------------------------------------------------------------
// Protocol constants
// Source: Camsense X1 community reverse-engineering wiki and USB captures.
//   https://github.com/Vidicon/camsense-X1 (packet format reference)
//
// Packet layout (36 bytes, fixed length):
//   [0..3]   sync header   0x55 0xAA 0x03 0x08
//   [4..5]   speed          LE uint16 (unit not officially documented;
//                           observed values ~300-700 correlate with RPM but
//                           exact scale factor is unverified. See note below.)
//   [6..7]   start_angle    LE uint16, decoded as raw/64.0 - 640.0 degrees
//   [8..31]  8 samples      each 3 bytes: LE uint16 range_mm, uint8 quality
//   [32..33] end_angle      LE uint16, same encoding as start_angle
//   [34..35] tail/checksum  2 bytes; unverified empirical capture / synthetic tests.
//
// Speed field note: field is decoded and exposed in diagnostics but NOT used
// for timing or sanity gating because the unit/scale is unverified.
// Timing model is instantaneous based on read completion; true acquisition timestamps
// and deskew are not available.
// ---------------------------------------------------------------------------
constexpr size_t kPacketSize = 36;
constexpr uint8_t kSyncByte0 = 0x55;
constexpr uint8_t kSyncByte1 = 0xAA;
constexpr uint8_t kSyncByte2 = 0x03;
constexpr uint8_t kSyncByte3 = 0x08;
constexpr int kSamplesPerPacket = 8;
constexpr int kBinsPerRevolution = 400;
constexpr double kIndexMultiplier = 400.0 / 360.0;
constexpr double kRangeMinM = 0.08;
constexpr double kRangeMaxM = 8.0;

// Angle sanity bounds (generous, based on capture observations).
constexpr double kAngleMinDeg = -40.0;
constexpr double kAngleMaxDeg = 400.0;
// Maximum angular span for one 8-sample packet.
constexpr double kMaxPacketSpanDeg = 15.0;

// Coverage threshold: fail-closed at 90%.
// Rationale: the Camsense X1 emits ~50 packets/rev × 8 samples = 400 bins.
// Normal operation fills most bins. Allowing 10% loss (360 bins)
// is generous enough for occasional USB glitches while rejecting scans with
// large missing sectors that would create ghost walls in SLAM.
constexpr int kMinUniqueBins = 360;
constexpr int kMinPacketsPerRevolution = 44;
// Maximum packets before forced reset (stuck-without-wrap leak detection).
constexpr int kMaxPacketsPerRevolution = 60;

// Continuity: maximum allowed angular jitter between consecutive packets.
// Packets arrive in monotonically increasing angle order; a packet whose
// start_angle (unwrapped) is less than or equal to the previous end_angle
// (unwrapped) by more than this tolerance is a duplicate or out-of-order.
constexpr double kMaxBackwardJitterDeg = 1.0;
// Forward gap tolerance: if the gap between consecutive packets exceeds this,
// it indicates a missing sector. We still accept the revolution but the
// coverage threshold will catch too-large gaps.
constexpr double kMaxForwardGapDeg = 30.0;

// ---------------------------------------------------------------------------
// Parsed packet result
// ---------------------------------------------------------------------------
struct ParsedPacket
{
  double start_angle_deg = 0.0;
  double end_angle_deg = 0.0;
  uint16_t speed_raw = 0;
  uint16_t tail_bytes = 0;
  struct Sample
  {
    uint16_t range_mm = 0;
    uint8_t quality = 0;
  };
  Sample samples[kSamplesPerPacket] = {};
};

// ---------------------------------------------------------------------------
// Scan result emitted on full revolution
// ---------------------------------------------------------------------------
struct ScanResult
{
  float ranges[kBinsPerRevolution] = {};
  float intensities[kBinsPerRevolution] = {};
  int unique_bins_filled = 0;
  int packets_in_revolution = 0;
  // Timing measured by injectable steady clock.
  double scan_duration_s = 0.0;
  // First-ray steady-clock timestamp (nanoseconds since epoch).
  // Node converts to ROS header.stamp at the publish boundary.
  int64_t first_ray_steady_ns = 0;
};

// ---------------------------------------------------------------------------
// Diagnostic counters
// ---------------------------------------------------------------------------
struct ParserCounters
{
  uint64_t bytes_received = 0;
  uint64_t bytes_discarded = 0;
  uint64_t candidates_rejected = 0;
  uint64_t packets_accepted = 0;
  uint64_t revolutions_published = 0;
  uint64_t revolutions_dropped = 0;
  uint64_t duplicates_detected = 0;
  uint64_t out_of_order_detected = 0;
  uint64_t continuity_resets = 0;
  uint64_t checksum_failures = 0;
  uint64_t tail_byte_sum = 0;
  uint64_t tail_byte_count = 0;
  uint64_t speed_sum = 0;
  uint64_t speed_count = 0;
  uint64_t proc_clock_fallbacks = 0;
  uint64_t gap_resets = 0;
  uint64_t nominal_fallbacks = 0;
  uint64_t stale_revolutions_dropped = 0;
  uint64_t cache_fallbacks = 0;
};

// ---------------------------------------------------------------------------
// Injectable steady clock
// ---------------------------------------------------------------------------
class SteadyClockInterface
{
public:
  virtual ~SteadyClockInterface() = default;
  virtual std::chrono::steady_clock::time_point now() const
  {
    return std::chrono::steady_clock::now();
  }
};

// ---------------------------------------------------------------------------
// PacketParser — byte-stream parser + revolution assembler
// ---------------------------------------------------------------------------
class PacketParser
{
public:
  using ScanCallback = std::function<void (const ScanResult &)>;

  explicit PacketParser(
    int baud_rate = 115200,
    std::atomic<int> * angle_offset_ptr = nullptr,
    ScanCallback on_scan = nullptr,
    std::shared_ptr<SteadyClockInterface> clock = nullptr);

  /// Feed raw bytes. May invoke on_scan zero or more times.
  void feed(const uint8_t * data, size_t len, int64_t rx_steady_ns = 0);

  /// Reset all state (call on serial session loss).
  void reset();

  /// Thread-safe counter snapshot.
  ParserCounters counters() const
  {
    std::lock_guard<std::mutex> lock(counters_mutex_);
    return counters_;
  }

  void set_scan_callback(ScanCallback cb) {on_scan_ = std::move(cb);}

private:
  static constexpr size_t kBufCapacity = 4096;
  uint8_t buf_[kBufCapacity] = {};
  int64_t buf_ts_[kBufCapacity] = {};
  size_t buf_len_ = 0;

  // Revolution assembly
  float rev_ranges_[kBinsPerRevolution] = {};
  float rev_intensities_[kBinsPerRevolution] = {};
  bool rev_bin_seen_[kBinsPerRevolution] = {};
  int rev_unique_bins_ = 0;
  int rev_packet_count_ = 0;
  bool rev_started_ = false;

  // Monotonic unwrapped angle tracking (L3-2).
  double rev_unwrapped_end_ = 0.0;

  // Timing (L3-3).
  std::shared_ptr<SteadyClockInterface> clock_;
  std::chrono::steady_clock::time_point rev_start_tp_;
  int64_t rev_first_ray_steady_ns_ = 0;
  int64_t rev_last_packet_steady_ns_ = 0;
  int64_t last_feed_steady_ns_ = 0;
  double last_good_scan_duration_ = -1.0;
  int64_t last_good_scan_duration_ns_stamp_ = 0;

  // Angle offset: read from external atomic owned by the node (L3-4).
  std::atomic<int> * angle_offset_ptr_ = nullptr;
  int rev_angle_offset_ = 0;  // Snapshot at start of revolution


  ScanCallback on_scan_;
  ParserCounters counters_;
  mutable std::mutex counters_mutex_;

  void try_parse();
  bool validate_candidate(const uint8_t * c, ParsedPacket & pkt);
  void apply_packet(const ParsedPacket & pkt, int64_t rx_steady_ns);
  void try_publish_revolution(int64_t end_steady_ns);
  void reset_revolution();
  void invalidate_timing_cache();
  void init_revolution_data();
  int angle_to_bin(double angle_deg) const;
  int current_angle_offset() const;
};

}  // namespace camsense_x1

#endif  // CAMSENSE_X1__CAMSENSE_X1_PARSER_HPP_
