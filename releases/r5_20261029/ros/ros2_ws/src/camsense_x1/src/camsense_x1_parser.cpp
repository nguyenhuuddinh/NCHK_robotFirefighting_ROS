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

#include "camsense_x1/camsense_x1_parser.hpp"

#include <algorithm>

namespace camsense_x1
{

PacketParser::PacketParser(
  int baud_rate,
  std::atomic<int> * angle_offset_ptr,
  ScanCallback on_scan,
  std::shared_ptr<SteadyClockInterface> clock)
: clock_(clock ? clock : std::make_shared<SteadyClockInterface>()),
  angle_offset_ptr_(angle_offset_ptr),
  on_scan_(std::move(on_scan))
{
  reset();
  (void)baud_rate;
}

void PacketParser::reset()
{
  buf_len_ = 0;
  rev_started_ = false;
  rev_packet_count_ = 0;
  rev_unique_bins_ = 0;
  rev_unwrapped_end_ = 0.0;
  last_good_scan_duration_ = -1.0;
  rev_first_ray_steady_ns_ = 0;
  invalidate_timing_cache();
  init_revolution_data();
}

void PacketParser::invalidate_timing_cache()
{
  last_good_scan_duration_ = -1.0;
  last_good_scan_duration_ns_stamp_ = 0;
}

void PacketParser::init_revolution_data()
{
  for (int i = 0; i < kBinsPerRevolution; ++i) {
    rev_ranges_[i] = std::numeric_limits<float>::infinity();
    rev_intensities_[i] = 0.0f;
    rev_bin_seen_[i] = false;
  }
  rev_unique_bins_ = 0;
}

// ---------------------------------------------------------------------------
// feed — append bytes to rolling buffer, invoke try_parse as data arrives
// ---------------------------------------------------------------------------
void PacketParser::feed(const uint8_t * data, size_t len, int64_t rx_steady_ns)
{
  if (rx_steady_ns <= 0) {
    rx_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      clock_->now().time_since_epoch()).count();
  }

  if (last_feed_steady_ns_ > 0) {
    int64_t delta_time_ns = rx_steady_ns - last_feed_steady_ns_;
    if (delta_time_ns < 0 || delta_time_ns > 500000000LL) {
      {
        std::lock_guard<std::mutex> lock(counters_mutex_);
        counters_.gap_resets++;
      }
      buf_len_ = 0;
      printf(
        "reset_revolution: unique_bins=%d, packet_count=%d\n", rev_unique_bins_,
        rev_packet_count_);
      invalidate_timing_cache();
      reset_revolution();
    }
  }
  last_feed_steady_ns_ = rx_steady_ns;

  uint64_t discarded = 0;

  for (size_t i = 0; i < len; ++i) {
    if (buf_len_ >= kBufCapacity) {
      std::memmove(buf_, buf_ + 1, buf_len_ - 1);
      std::memmove(buf_ts_, buf_ts_ + 1, (buf_len_ - 1) * sizeof(int64_t));
      buf_len_--;
      discarded++;
    }
    buf_[buf_len_] = data[i];
    // Instantaneous timing model: all bytes in this read get the read completion time.
    buf_ts_[buf_len_] = rx_steady_ns;
    buf_len_++;
  }

  try_parse();

  {
    std::lock_guard<std::mutex> lock(counters_mutex_);
    counters_.bytes_received += len;
    counters_.bytes_discarded += discarded;
  }
}

// ---------------------------------------------------------------------------
// try_parse — attempt to extract 36-byte packets from buffer
// ---------------------------------------------------------------------------
void PacketParser::try_parse()
{
  while (buf_len_ >= kPacketSize) {
    // Seek sync at position 0.
    if (buf_[0] != kSyncByte0 ||
      buf_[1] != kSyncByte1 ||
      buf_[2] != kSyncByte2 ||
      buf_[3] != kSyncByte3)
    {
      // Scan forward for next sync.
      size_t skip = 1;
      for (size_t s = 1; s + 3 < buf_len_; ++s) {
        if (buf_[s] == kSyncByte0 &&
          buf_[s + 1] == kSyncByte1 &&
          buf_[s + 2] == kSyncByte2 &&
          buf_[s + 3] == kSyncByte3)
        {
          skip = s;
          break;
        }
        skip = s + 1;
      }
      if (skip > 0 && skip <= buf_len_) {
        {
          std::lock_guard<std::mutex> lock(counters_mutex_);
          counters_.bytes_discarded += skip;
        }
        std::memmove(buf_, buf_ + skip, buf_len_ - skip);
        std::memmove(buf_ts_, buf_ts_ + skip, (buf_len_ - skip) * sizeof(int64_t));
        buf_len_ -= skip;
      }
      continue;
    }

    // Sync at [0..3] and >= 36 bytes. Validate candidate.
    ParsedPacket pkt;
    if (validate_candidate(buf_, pkt)) {
      int64_t packet_steady_ns = buf_ts_[kPacketSize - 1];
      {
        std::lock_guard<std::mutex> lock(counters_mutex_);
        counters_.packets_accepted++;
        counters_.tail_byte_sum += pkt.tail_bytes;
        counters_.tail_byte_count++;
        counters_.speed_sum += pkt.speed_raw;
        counters_.speed_count++;
      }
      apply_packet(pkt, packet_steady_ns);
      std::memmove(buf_, buf_ + kPacketSize, buf_len_ - kPacketSize);
      std::memmove(buf_ts_, buf_ts_ + kPacketSize, (buf_len_ - kPacketSize) * sizeof(int64_t));
      buf_len_ -= kPacketSize;
    } else {
      // Bad candidate: drop one byte and resync.
      {
        std::lock_guard<std::mutex> lock(counters_mutex_);
        counters_.candidates_rejected++;
        counters_.bytes_discarded++;
      }
      std::memmove(buf_, buf_ + 1, buf_len_ - 1);
      std::memmove(buf_ts_, buf_ts_ + 1, (buf_len_ - 1) * sizeof(int64_t));
      buf_len_--;
    }
  }
}

// ---------------------------------------------------------------------------
// validate_candidate — decode angles and check sanity; NO payload-content
//   heuristics (L3-1). Validation is: header + angle range + span + checksum.
// ---------------------------------------------------------------------------
bool PacketParser::validate_candidate(const uint8_t * c, ParsedPacket & pkt)
{
  // Camsense V3.0 Checksum (L3-1).
  uint32_t accumulator = 0;
  for (int i = 0; i < 17; ++i) {
    uint16_t word = static_cast<uint16_t>(c[2 * i + 1]) << 8 | c[2 * i];
    accumulator = (accumulator << 1) + word;
  }
  uint16_t expected_checksum = ((accumulator & 0x7FFF) + (accumulator >> 15)) & 0x7FFF;
  uint16_t actual_checksum = static_cast<uint16_t>(c[35]) << 8 | c[34];

  if (expected_checksum != actual_checksum) {
    std::lock_guard<std::mutex> lock(counters_mutex_);
    counters_.checksum_failures++;
    return false;
  }

  // Decode fields.
  pkt.speed_raw = static_cast<uint16_t>(c[5]) << 8 | c[4];
  uint16_t sa_raw = static_cast<uint16_t>(c[7]) << 8 | c[6];
  pkt.start_angle_deg = sa_raw / 64.0 - 640.0;
  uint16_t ea_raw = static_cast<uint16_t>(c[33]) << 8 | c[32];
  pkt.end_angle_deg = ea_raw / 64.0 - 640.0;
  pkt.tail_bytes = actual_checksum;

  // Angle sanity.
  if (pkt.start_angle_deg < kAngleMinDeg ||
    pkt.start_angle_deg > kAngleMaxDeg)
  {
    return false;
  }
  if (pkt.end_angle_deg < kAngleMinDeg ||
    pkt.end_angle_deg > kAngleMaxDeg)
  {
    return false;
  }

  // Span sanity.
  double span = pkt.end_angle_deg - pkt.start_angle_deg;
  if (span < 0.0) {
    span += 360.0;
  }
  if (span > kMaxPacketSpanDeg || span < 0.1) {
    return false;
  }

  // Decode samples (L4).
  for (int i = 0; i < kSamplesPerPacket; ++i) {
    int j = 8 + 3 * i;
    uint8_t low = c[j];
    uint8_t high = c[j + 1];
    bool is_invalid = (high & 0x80) != 0;
    uint16_t range = ((high & 0x3F) << 8) | low;
    pkt.samples[i].range_mm = is_invalid ? 0 : range;
    pkt.samples[i].quality = c[j + 2];
  }

  return true;
}

// ---------------------------------------------------------------------------
// apply_packet — revolution assembly with monotonic unwrapped progression
// ---------------------------------------------------------------------------
void PacketParser::apply_packet(const ParsedPacket & pkt, int64_t rx_steady_ns)
{
  double cur_start = pkt.start_angle_deg;

  // --- Timing discontinuity detection ---
  if (rev_started_) {
    int64_t delta_time_ns = rx_steady_ns - rev_last_packet_steady_ns_;
    // Threshold: 500ms. Rationale: normal revolution is ~165ms.
    // Gap > 500ms means missing multiple revolutions, invalidating the current state.
    // Backward timestamp also invalidates the cache.
    if (delta_time_ns < 0 || delta_time_ns > 500000000LL) {
      {
        std::lock_guard<std::mutex> lock(counters_mutex_);
        counters_.gap_resets++;
      }
      printf(
        "reset_revolution: unique_bins=%d, packet_count=%d\n", rev_unique_bins_,
        rev_packet_count_);
      invalidate_timing_cache();
      reset_revolution();
    }
  }

  // --- Wrap / continuity detection ---
  bool is_wrap = false;
  if (rev_started_) {
    // Unwrap: if current start is much less than last unwrapped end,
    // it means we crossed the 360->0 boundary.
    double last_uw = rev_unwrapped_end_;
    double cur_uw = cur_start;
    // If last was >300 and current <60, this is a genuine wrap.
    if (last_uw > 300.0 && cur_uw < 60.0) {
      is_wrap = true;
    } else {
      double delta = cur_uw - last_uw;
      if (delta < -kMaxBackwardJitterDeg) {
        // Backward motion: duplicate or out-of-order.
        {
          std::lock_guard<std::mutex> lock(counters_mutex_);
          if (std::abs(delta) < 1.5) {
            counters_.duplicates_detected++;
          } else {
            counters_.out_of_order_detected++;
          }
          counters_.continuity_resets++;
        }
        printf(
          "reset_revolution: unique_bins=%d, packet_count=%d\n", rev_unique_bins_,
          rev_packet_count_);
        invalidate_timing_cache();
        reset_revolution();
        // Start fresh with this packet.
      } else if (delta > kMaxForwardGapDeg) {
        // Large forward gap (missing sector). Accept but coverage will gate.
      }
      // delta in [-jitter, +gap]: normal progression, accept.
    }
  }

  if (is_wrap && rev_started_) {
    try_publish_revolution(rx_steady_ns);
  }

  if (!rev_started_) {
    rev_started_ = true;
    rev_start_tp_ = clock_->now();
    rev_first_ray_steady_ns_ = (rx_steady_ns > 0) ? rx_steady_ns :
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      rev_start_tp_.time_since_epoch()).count();
    rev_packet_count_ = 0;
    rev_unwrapped_end_ = 0.0;
    rev_angle_offset_ = current_angle_offset();
    init_revolution_data();
  }

  // Too-many-packets leak detection.
  if (rev_packet_count_ >= kMaxPacketsPerRevolution) {
    {
      std::lock_guard<std::mutex> lock(counters_mutex_);
      counters_.revolutions_dropped++;
      counters_.continuity_resets++;
    }
    printf(
      "reset_revolution: unique_bins=%d, packet_count=%d\n", rev_unique_bins_,
      rev_packet_count_);
    invalidate_timing_cache();
    reset_revolution();
    rev_started_ = true;
    rev_start_tp_ = clock_->now();
    rev_first_ray_steady_ns_ = (rx_steady_ns > 0) ? rx_steady_ns :
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      rev_start_tp_.time_since_epoch()).count();
    rev_packet_count_ = 0;
    rev_unwrapped_end_ = 0.0;
    rev_angle_offset_ = current_angle_offset();
    init_revolution_data();
  }

  // Compute per-sample angular resolution (L4).
  double span = pkt.end_angle_deg - pkt.start_angle_deg;
  if (span < 0.0) {
    span += 360.0;
  }
  double angle_res = span / static_cast<double>(kSamplesPerPacket - 1);

  int offset = rev_angle_offset_;

  for (int i = 0; i < kSamplesPerPacket; ++i) {
    double measured_angle = pkt.start_angle_deg + angle_res * i - offset;
    int bin = angle_to_bin(measured_angle);

    if (!rev_bin_seen_[bin]) {
      rev_bin_seen_[bin] = true;
      rev_unique_bins_++;
    }

    uint16_t range_mm = pkt.samples[i].range_mm;
    uint8_t quality = pkt.samples[i].quality;

    if (range_mm == 0 || quality == 0) {
      rev_ranges_[bin] = std::numeric_limits<float>::infinity();
      rev_intensities_[bin] = 0.0f;
    } else {
      float range_m = static_cast<float>(range_mm) / 1000.0f;
      if (range_m < static_cast<float>(kRangeMinM) ||
        range_m > static_cast<float>(kRangeMaxM))
      {
        rev_ranges_[bin] = std::numeric_limits<float>::infinity();
        rev_intensities_[bin] = 0.0f;
      } else {
        rev_ranges_[bin] = range_m;
        rev_intensities_[bin] = static_cast<float>(quality);
      }
    }
  }

  rev_packet_count_++;
  // Track unwrapped end angle for monotonic progression.
  double end_uw = pkt.end_angle_deg;
  if (end_uw < cur_start) {
    end_uw += 360.0;  // Handle wrap within packet.
  }
  rev_unwrapped_end_ = end_uw;
  rev_last_packet_steady_ns_ = rx_steady_ns;
}

// ---------------------------------------------------------------------------
// try_publish_revolution — check coverage and emit ScanResult
// ---------------------------------------------------------------------------
void PacketParser::try_publish_revolution(int64_t end_steady_ns)
{
  double duration_s = 0.0;
  if (rev_first_ray_steady_ns_ > 0 && end_steady_ns > rev_first_ray_steady_ns_) {
    duration_s = static_cast<double>(end_steady_ns - rev_first_ray_steady_ns_) / 1e9;
  }

  auto now_tp = clock_->now();
  double proc_duration = std::chrono::duration<double>(now_tp - rev_start_tp_).count();

  // If the total gathered duration exceeds 2 seconds, it's stale and should be dropped.
  if (duration_s > 2.0 || proc_duration > 2.0) {
    {
      std::lock_guard<std::mutex> lock(counters_mutex_);
      counters_.stale_revolutions_dropped++;
      counters_.revolutions_dropped++;
    }
    printf(
      "reset_revolution: unique_bins=%d, packet_count=%d\n", rev_unique_bins_,
      rev_packet_count_);
    invalidate_timing_cache();
    reset_revolution();
    return;
  }

  bool duration_ok = (duration_s > 0.01 && duration_s < 2.0);
  double scan_time;
  enum class TimingMode { MEASURED, PROC, CACHE, NOMINAL };
  TimingMode timing_mode = TimingMode::MEASURED;

  if (duration_ok) {
    scan_time = duration_s;
  } else {
    if (proc_duration > 0.01 && proc_duration < 2.0) {
      scan_time = proc_duration;
      timing_mode = TimingMode::PROC;
    } else {
      bool use_cache = last_good_scan_duration_ > 0.0 &&
        (end_steady_ns - last_good_scan_duration_ns_stamp_) >= 0 &&
        (end_steady_ns - last_good_scan_duration_ns_stamp_) < 1000000000LL;
      if (use_cache) {
        scan_time = last_good_scan_duration_;
        timing_mode = TimingMode::CACHE;
      } else {
        scan_time = 0.165;
        timing_mode = TimingMode::NOMINAL;
      }
    }
  }

  if (rev_unique_bins_ >= kMinUniqueBins &&
    rev_packet_count_ >= kMinPacketsPerRevolution)
  {
    if (duration_ok) {
      last_good_scan_duration_ = duration_s;
      last_good_scan_duration_ns_stamp_ = end_steady_ns;
    }

    ScanResult result;
    std::memcpy(result.ranges, rev_ranges_, sizeof(rev_ranges_));
    std::memcpy(result.intensities, rev_intensities_, sizeof(rev_intensities_));
    result.unique_bins_filled = rev_unique_bins_;
    result.packets_in_revolution = rev_packet_count_;
    result.scan_duration_s = scan_time;
    result.first_ray_steady_ns = rev_first_ray_steady_ns_;

    {
      std::lock_guard<std::mutex> lock(counters_mutex_);
      counters_.revolutions_published++;
      if (timing_mode == TimingMode::PROC) {
        counters_.proc_clock_fallbacks++;
      } else if (timing_mode == TimingMode::CACHE) {
        counters_.cache_fallbacks++;
      } else if (timing_mode == TimingMode::NOMINAL) {
        counters_.nominal_fallbacks++;
      }
    }
    if (on_scan_) {
      on_scan_(result);
    }
  } else {
    std::lock_guard<std::mutex> lock(counters_mutex_);
    counters_.revolutions_dropped++;
  }

  printf(
    "reset_revolution: unique_bins=%d, packet_count=%d\n", rev_unique_bins_,
    rev_packet_count_); reset_revolution();
}

void PacketParser::reset_revolution()
{
  rev_started_ = false;
  rev_packet_count_ = 0;
  rev_unique_bins_ = 0;
  rev_unwrapped_end_ = 0.0;
  rev_first_ray_steady_ns_ = 0;
  init_revolution_data();
}

int PacketParser::angle_to_bin(double angle_deg) const
{
  int idx = static_cast<int>(std::round(angle_deg * kIndexMultiplier));
  idx %= kBinsPerRevolution;
  if (idx < 0) {
    idx += kBinsPerRevolution;
  }
  return idx;
}

int PacketParser::current_angle_offset() const
{
  if (angle_offset_ptr_) {
    return angle_offset_ptr_->load(std::memory_order_relaxed);
  }
  return 0;
}

}  // namespace camsense_x1
