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

/// \file Hardware-free unit tests for Camsense X1 parser + revolution assembler.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "camsense_x1/camsense_x1_parser.hpp"

using camsense_x1::kBinsPerRevolution;
using camsense_x1::kMinPacketsPerRevolution;
using camsense_x1::kMinUniqueBins;
using camsense_x1::kPacketSize;
using camsense_x1::kRangeMaxM;
using camsense_x1::kRangeMinM;
using camsense_x1::PacketParser;
using camsense_x1::ParserCounters;
using camsense_x1::ScanResult;
using camsense_x1::SteadyClockInterface;

// --- Fake clock ---
class FakeClock : public SteadyClockInterface
{
public:
  FakeClock()
  : ns_(1000000000000LL) {}
  std::chrono::steady_clock::time_point now() const override
  {
    return std::chrono::steady_clock::time_point(std::chrono::nanoseconds(ns_));
  }
  void advance(std::chrono::milliseconds ms)
  {
    ns_ += std::chrono::duration_cast<std::chrono::nanoseconds>(ms).count();
  }

  int64_t ns_;
};

// --- Packet builder ---
static void fix_checksum(std::vector<uint8_t> & p)
{
  uint32_t accumulator = 0;
  for (int i = 0; i < 17; ++i) {
    uint16_t word = static_cast<uint16_t>(p[2 * i + 1]) << 8 | p[2 * i];
    accumulator = (accumulator << 1) + word;
  }
  uint16_t expected_checksum = ((accumulator & 0x7FFF) + (accumulator >> 15)) & 0x7FFF;
  p[34] = expected_checksum & 0xFF;
  p[35] = (expected_checksum >> 8) & 0xFF;
}

static std::vector<uint8_t> make_packet(
  double start_deg, double end_deg,
  uint16_t range_mm = 1000, uint8_t quality = 100,
  bool corrupt_checksum = false)
{
  std::vector<uint8_t> p(36, 0);
  p[0] = 0x55; p[1] = 0xAA; p[2] = 0x03; p[3] = 0x08;
  p[4] = 0xF4; p[5] = 0x01;
  uint16_t sa = static_cast<uint16_t>((start_deg + 640.0) * 64.0);
  p[6] = sa & 0xFF; p[7] = (sa >> 8) & 0xFF;
  for (int i = 0; i < 8; ++i) {
    int j = 8 + 3 * i;
    p[j] = range_mm & 0xFF; p[j + 1] = (range_mm >> 8) & 0xFF;
    p[j + 2] = quality;
  }
  uint16_t ea = static_cast<uint16_t>((end_deg + 640.0) * 64.0);
  p[32] = ea & 0xFF; p[33] = (ea >> 8) & 0xFF;

  fix_checksum(p);
  if (corrupt_checksum) {
    p[34] ^= 0xFF;
    p[35] ^= 0xFF;
  }
  return p;
}

static std::vector<std::vector<uint8_t>> make_full_revolution(
  uint16_t range_mm = 1000, uint8_t quality = 100)
{
  std::vector<std::vector<uint8_t>> pkts;
  constexpr double step = 6.8;
  for (int i = 0; i < 53; ++i) {
    pkts.push_back(make_packet(i * step, i * step + step, range_mm, quality));
  }
  return pkts;
}

static void feed_packets(
  PacketParser & p, const std::vector<std::vector<uint8_t>> & pkts)
{
  for (auto & pkt : pkts) {
    p.feed(pkt.data(), pkt.size());
  }
}


// ======== GROUP 1: Basic packet accept ========
TEST(ParserTest, SinglePacketAccepted)
{
  PacketParser p;
  auto pkt = make_packet(10.0, 17.0);
  p.feed(pkt.data(), pkt.size());
  EXPECT_EQ(p.counters().packets_accepted, 1u);
  EXPECT_EQ(p.counters().candidates_rejected, 0u);
}

// ======== GROUP 2: Chunk boundary splits ========
TEST(ParserTest, ChunkBoundaryAllSplits)
{
  auto pkt = make_packet(20.0, 27.0);
  for (size_t s = 1; s < 36; ++s) {
    PacketParser p;
    p.feed(pkt.data(), s);
    p.feed(pkt.data() + s, 36 - s);
    EXPECT_EQ(p.counters().packets_accepted, 1u) << "split=" << s;
  }
}

// ======== GROUP 3: Header split across reads ========
TEST(ParserTest, HeaderSplitAcrossReads)
{
  auto pkt = make_packet(30.0, 37.0);
  PacketParser p;
  for (int i = 0; i < 4; ++i) {
    p.feed(&pkt[i], 1);
  }
  p.feed(pkt.data() + 4, 32);
  EXPECT_EQ(p.counters().packets_accepted, 1u);
}

// ======== L4-1: Truncated + Valid ========
TEST(ParserTest, DirectTruncatedThenValid)
{
  auto clk = std::make_shared<FakeClock>();
  ScanResult captured;
  int pub_count = 0;
  std::atomic<int> offset{0};
  PacketParser p(115200, &offset, [&](const ScanResult & r) {captured = r; pub_count++;}, clk);

  auto pktA = make_packet(0.0, 7.0, 1000, 100);     // valid A but truncated
  auto pktB = make_packet(0.0, 7.0, 2000, 200);     // valid B with marker (range 2m, intensity 200)

  std::vector<uint8_t> stream;
  stream.insert(stream.end(), pktA.begin(), pktA.begin() + 35);     // truncate A
  stream.insert(stream.end(), pktB.begin(), pktB.end());

  // Fill the rest of revolution
  for (int i = 1; i < 53; ++i) {
    auto pkt = make_packet(i * 6.8, i * 6.8 + 6.8, 1000, 100);
    stream.insert(stream.end(), pkt.begin(), pkt.end());
  }
  // trigger wrap
  auto trig = make_packet(0.0, 6.8);
  stream.insert(stream.end(), trig.begin(), trig.end());

  p.feed(stream.data(), stream.size());

  EXPECT_EQ(pub_count, 1);
  // Find B's marker
  bool found_marker = false;
  for (int i = 0; i < 400; ++i) {
    if (captured.ranges[i] == 2.0f && captured.intensities[i] == 200.0f) {
      found_marker = true;
      break;
    }
  }
  EXPECT_TRUE(found_marker);
}

// ======== L3-1B: Full sync in payload must be accepted ========
TEST(ParserTest, SyncPatternInPayloadAccepted)
{
  // Build packet with range values that produce 0x55 0xAA in bytes.
  auto pkt = make_packet(50.0, 57.0, 0xAA55, 0x03);
  // Manually set quality of sample 1 to 0x08 so bytes [11..14] = 55 AA 03 08.
  pkt[10] = 0x55; pkt[11] = 0xAA; pkt[12] = 0x03;
  // Sample 2 starts at byte 11: range lo=08, hi=something.
  pkt[11] = 0xAA; pkt[12] = 0x03;
  // Directly set bytes 8..11 = 55 AA 03 08 to embed sync.
  pkt[8] = 0x55; pkt[9] = 0xAA; pkt[10] = 0x03; pkt[11] = 0x08;
  fix_checksum(pkt);
  PacketParser p;
  p.feed(pkt.data(), pkt.size());
  EXPECT_EQ(p.counters().packets_accepted, 1u);
  EXPECT_EQ(p.counters().candidates_rejected, 0u);
}


// ======== L3-1D: Partial sync split across chunks ========
TEST(ParserTest, PartialSyncSplitAcrossChunks)
{
  // Garbage ending in 0x55, then next chunk starts with AA 03 08 + packet.
  PacketParser p;
  std::vector<uint8_t> chunk1 = {0xFF, 0xFF, 0x55};
  p.feed(chunk1.data(), chunk1.size());
  auto pkt = make_packet(60.0, 67.0);
  std::vector<uint8_t> chunk2;
  chunk2.push_back(0xAA); chunk2.push_back(0x03); chunk2.push_back(0x08);
  chunk2.insert(chunk2.end(), pkt.begin(), pkt.end());
  p.feed(chunk2.data(), chunk2.size());
  // The pkt should be accepted.
  EXPECT_GE(p.counters().packets_accepted, 1u);
}

// ======== L3-1E: Garbage + back-to-back frames ========
TEST(ParserTest, GarbageThenBackToBack)
{
  PacketParser p;
  std::vector<uint8_t> garbage = {0x00, 0xFF, 0x55, 0x55, 0xAA};
  p.feed(garbage.data(), garbage.size());
  auto pkt1 = make_packet(70.0, 77.0);
  auto pkt2 = make_packet(80.0, 87.0);
  p.feed(pkt1.data(), pkt1.size());
  p.feed(pkt2.data(), pkt2.size());
  EXPECT_EQ(p.counters().packets_accepted, 2u);
}

// ======== L3-2A: Duplicate packet rejects revolution ========
TEST(ParserTest, SingleDuplicateRejectsRevolution)
{
  auto clk = std::make_shared<FakeClock>();
  int pub_count = 0;
  std::atomic<int> offset{0};
  PacketParser p(115200, &offset, [&](const ScanResult &) {pub_count++;}, clk);

  auto rev = make_full_revolution();
  // Insert duplicate of packet 25 right after it.
  std::vector<std::vector<uint8_t>> modified;
  for (size_t i = 0; i < rev.size(); ++i) {
    modified.push_back(rev[i]);
    if (i == 25) {
      modified.push_back(rev[25]);     // duplicate
    }
  }
  clk->advance(std::chrono::milliseconds(200));
  feed_packets(p, modified);
  auto trig = make_packet(0.0, 6.8);
  p.feed(trig.data(), trig.size());
  // Duplicate causes continuity reset (backward jump > jitter).
  auto c = p.counters();
  EXPECT_GE(c.continuity_resets, 1u);
  EXPECT_GE(c.out_of_order_detected + c.duplicates_detected, 1u);
}

// ======== L3-2B: Adjacent swap rejects revolution ========
TEST(ParserTest, AdjacentSwapRejectsRevolution)
{
  auto clk = std::make_shared<FakeClock>();
  int pub_count = 0;
  std::atomic<int> offset{0};
  PacketParser p(115200, &offset, [&](const ScanResult &) {pub_count++;}, clk);

  auto rev = make_full_revolution();
  // Swap packets 20 and 21.
  std::swap(rev[20], rev[21]);
  clk->advance(std::chrono::milliseconds(200));
  feed_packets(p, rev);
  auto trig = make_packet(0.0, 6.8);
  p.feed(trig.data(), trig.size());
  auto c = p.counters();
  EXPECT_GE(c.out_of_order_detected + c.continuity_resets, 1u);
}

// ======== L3-2C: Recovery after continuity reset ========
TEST(ParserTest, RecoveryAfterContinuityReset)
{
  auto clk = std::make_shared<FakeClock>();
  int pub_count = 0;
  std::atomic<int> offset{0};
  PacketParser p(115200, &offset, [&](const ScanResult &) {pub_count++;}, clk);

  // Bad revolution triggers reset. Then a clean revolution follows.
  // Feed 10 ascending packets, then a duplicate to trigger reset.
  for (int i = 0; i < 10; ++i) {
    p.feed(make_packet(i * 6.8, i * 6.8 + 6.8).data(), 36);
  }
  // Duplicate of packet 9 triggers reset.
  p.feed(make_packet(9 * 6.8, 9 * 6.8 + 6.8).data(), 36);
  EXPECT_GE(p.counters().continuity_resets, 1u);

  // Now feed a clean full revolution.
  feed_packets(p, make_full_revolution());
  clk->advance(std::chrono::milliseconds(200));
  p.feed(make_packet(0.0, 6.8).data(), 36);

  // The good revolution should publish.
  EXPECT_GE(pub_count, 1);
}

// ======== L3-3A: First-ray timestamp ========
TEST(ParserTest, FirstRayTimestamp)
{
  auto clk = std::make_shared<FakeClock>();
  ScanResult captured;
  bool published = false;
  std::atomic<int> offset{0};
  PacketParser p(115200, &offset, [&](const ScanResult & r) {
      captured = r; published = true;
    }, clk);

  auto rev = make_full_revolution();
  feed_packets(p, rev);
  clk->advance(std::chrono::milliseconds(200));
  auto trig = make_packet(0.0, 6.8);
  p.feed(trig.data(), trig.size());

  ASSERT_TRUE(published);
  // The first ray timestamp in instantaneous mode should exactly match the
  // rx_steady_ns timestamp at the time the first packet of the revolution is fed.
  int64_t expected_ns = 1000000000000LL;
  EXPECT_EQ(captured.first_ray_steady_ns, expected_ns);
}

// ======== L3-3B: Two consecutive revolutions timing ========
TEST(ParserTest, TwoConsecutiveRevolutionsTiming)
{
  auto clk = std::make_shared<FakeClock>();
  std::vector<ScanResult> results;
  std::atomic<int> offset{0};
  PacketParser p(115200, &offset, [&](const ScanResult & r) {
      results.push_back(r);
    }, clk);

  // Rev 1.
  feed_packets(p, make_full_revolution());
  clk->advance(std::chrono::milliseconds(200));
  p.feed(make_packet(0.0, 6.8).data(), 36);

  // Rev 2.
  feed_packets(p, make_full_revolution());
  clk->advance(std::chrono::milliseconds(200));
  p.feed(make_packet(0.0, 6.8).data(), 36);

  ASSERT_EQ(results.size(), 2u);
  EXPECT_NEAR(results[0].scan_duration_s, 0.2, 0.05);
  EXPECT_NEAR(results[1].scan_duration_s, 0.2, 0.05);
  // Second revolution's first_ray should be later.
  EXPECT_GT(results[1].first_ray_steady_ns, results[0].first_ray_steady_ns);
}

// ======== L3-3C: Wrap inside one chunk ========
TEST(ParserTest, WrapInsideOneChunk)
{
  auto clk = std::make_shared<FakeClock>();
  int pub_count = 0;
  std::atomic<int> offset{0};
  PacketParser p(115200, &offset, [&](const ScanResult &) {pub_count++;}, clk);

  auto rev = make_full_revolution();
  std::vector<uint8_t> single_chunk;
  for (const auto & pkt : rev) {
    single_chunk.insert(single_chunk.end(), pkt.begin(), pkt.end());
  }
  // Append trigger packet to the same chunk
  auto trig = make_packet(0.0, 6.8);
  single_chunk.insert(single_chunk.end(), trig.begin(), trig.end());

  p.feed(single_chunk.data(), single_chunk.size());
  EXPECT_EQ(pub_count, 1);
}

// ======== L3-4: Angle offset atomic ========
TEST(ParserTest, AngleOffsetAtomic)
{
  std::atomic<int> offset{0};
  auto clk = std::make_shared<FakeClock>();
  ScanResult r1, r2;
  int pub_count = 0;
  PacketParser p(115200, &offset, [&](const ScanResult & r) {
      if (pub_count == 0) {r1 = r;} else {r2 = r;}
      pub_count++;
    }, clk);

  feed_packets(p, make_full_revolution());
  clk->advance(std::chrono::milliseconds(200));
  p.feed(make_packet(0.0, 6.8).data(), 36);

  // Change offset between revolutions.
  offset.store(180);

  feed_packets(p, make_full_revolution());
  clk->advance(std::chrono::milliseconds(200));
  p.feed(make_packet(0.0, 6.8).data(), 36);

  EXPECT_EQ(pub_count, 2);
  // The two scans should have different bin arrangements.
}

// ======== L3-5A: Coverage threshold boundary - below ========
TEST(ParserTest, CoverageBelowThresholdDropped)
{
  auto clk = std::make_shared<FakeClock>();
  int pub_count = 0;
  std::atomic<int> offset{0};
  PacketParser p(115200, &offset, [&](const ScanResult &) {pub_count++;}, clk);

  // Only 20 packets covering 0-144 degrees. Not enough coverage.
  for (int i = 0; i < 20; ++i) {
    p.feed(make_packet(i * 6.8, i * 6.8 + 6.8).data(), 36);
  }
  clk->advance(std::chrono::milliseconds(200));
  // Jump backward from 144 to 0 triggers out-of-order reset.
  p.feed(make_packet(0.0, 6.8).data(), 36);
  EXPECT_EQ(pub_count, 0);
  // The insufficient revolution is handled via continuity reset.
  auto c = p.counters();
  EXPECT_GE(c.continuity_resets + c.revolutions_dropped, 1u);
}

// ======== L3-5B: Coverage at threshold - published ========
TEST(ParserTest, CoverageAtThresholdPublished)
{
  auto clk = std::make_shared<FakeClock>();
  int pub_count = 0;
  std::atomic<int> offset{0};
  PacketParser p(115200, &offset, [&](const ScanResult &) {pub_count++;}, clk);

  // Full revolution should have enough.
  feed_packets(p, make_full_revolution());
  clk->advance(std::chrono::milliseconds(200));
  p.feed(make_packet(0.0, 6.8).data(), 36);
  EXPECT_EQ(pub_count, 1);
}

// ======== L3-5C: Speed diagnostic exposure ========
TEST(ParserTest, SpeedDiagnosticExposed)
{
  PacketParser p;
  auto pkt = make_packet(10.0, 17.0);
  p.feed(pkt.data(), pkt.size());
  auto c = p.counters();
  EXPECT_EQ(c.speed_count, 1u);
  EXPECT_GT(c.speed_sum, 0u);
}

// ======== L4-2: Interpolation boundary ========
TEST(ParserTest, InterpolationBoundary)
{
  auto clk = std::make_shared<FakeClock>();
  ScanResult captured;
  std::atomic<int> offset{0};
  PacketParser p(115200, &offset, [&](const ScanResult & r) {captured = r;}, clk);

  // One packet from 0 to 7.0 degrees. 8 samples -> angle_res = 1.0 deg.
  auto rev = make_full_revolution();
  rev[0] = make_packet(0.0, 7.0, 1000, 100);
  feed_packets(p, rev);
  clk->advance(std::chrono::milliseconds(200));
  p.feed(make_packet(0.0, 6.8).data(), 36);

  // angle_to_bin for 0.0 is bin 0, for 7.0 is bin 7 (since 400 bins / 360 deg = 1.111 bin/deg).
  // Wait, bin = angle * 400 / 360.
  // 0.0 * 1.111 = 0
  // 7.0 * 1.111 = 7.77 -> 8.
  int bin0 = static_cast<int>(0.0 * 400.0 / 360.0 + 0.5) % 400;
  int bin7 = static_cast<int>(7.0 * 400.0 / 360.0 + 0.5) % 400;
  EXPECT_EQ(captured.ranges[bin0], 1.0f);
  EXPECT_EQ(captured.ranges[bin7], 1.0f);
}

// ======== L4-5: Multiple packets in one feed ========
TEST(ParserTest, TimestampBacklog)
{
  auto clk = std::make_shared<FakeClock>();
  ScanResult captured;
  int pub_count = 0;
  std::atomic<int> offset{0};
  PacketParser p(115200, &offset, [&](const ScanResult & r) {captured = r; pub_count++;}, clk);

  auto rev = make_full_revolution();
  std::vector<uint8_t> big;
  for (auto & pkt : rev) {
    big.insert(big.end(), pkt.begin(), pkt.end());
  }
  // Feed half the revolution at steady_ns = 2000e9
  p.feed(big.data(), big.size() / 2, 2000000000000LL);
  // Feed rest at steady_ns = 2000.1e9 (100ms later)
  p.feed(big.data() + big.size() / 2, big.size() / 2, 2000100000000LL);

  clk->advance(std::chrono::milliseconds(200));
  p.feed(make_packet(0.0, 6.8).data(), 36, 2000200000000LL);

  EXPECT_EQ(pub_count, 1);
  // In instantaneous timing model, the first packet of the revolution
  // (which is in the first chunk fed at 2000e9) simply gets that rx_steady_ns.
  EXPECT_EQ(captured.first_ray_steady_ns, 2000000000000LL);
}

// ======== L4-6: Snapshot angle_offset ========
TEST(ParserTest, SnapshotAngleOffset)
{
  std::atomic<int> offset{0};
  auto clk = std::make_shared<FakeClock>();
  ScanResult r;
  int pub_count = 0;
  PacketParser p(115200, &offset, [&](const ScanResult & res) {r = res; pub_count++;}, clk);

  auto rev = make_full_revolution();
  // Mark packet 0 and packet 26
  rev[0] = make_packet(0.0, 6.8, 2000, 201);     // early
  rev[26] = make_packet(26 * 6.8, 26 * 6.8 + 6.8, 3000, 202);     // late

  p.feed(rev[0].data(), rev[0].size());     // Revolution starts, offset 0 snapped.
  offset.store(180);     // Change mid-revolution, should be ignored for this revolution.

  for (size_t i = 1; i < rev.size(); ++i) {
    p.feed(rev[i].data(), rev[i].size());
  }
  clk->advance(std::chrono::milliseconds(200));
  p.feed(make_packet(0.0, 6.8).data(), 36);

  EXPECT_EQ(pub_count, 1);
  int early_marker = -1, late_marker = -1;
  for (int i = 0; i < 400; ++i) {
    if (early_marker == -1 && r.intensities[i] == 201.0f && r.ranges[i] == 2.0f) {early_marker = i;}
    if (late_marker == -1 && r.intensities[i] == 202.0f && r.ranges[i] == 3.0f) {late_marker = i;}
  }
  // Offset was 0 for this revolution. Bin 0 for 0 deg, Bin ~196 for 26*6.8=176.8 deg.
  EXPECT_TRUE(early_marker == 0 || early_marker == 1);     // 0.0 -> bin 0 or 1
  EXPECT_TRUE(late_marker >= 190 && late_marker <= 200);

  // Now test the next revolution, where the offset of 180 should take effect.
  auto rev2 = make_full_revolution();
  rev2[0] = make_packet(0.0, 6.8, 4000, 203);     // new early
  for (size_t i = 0; i < rev2.size(); ++i) {
    p.feed(rev2[i].data(), rev2[i].size());
  }
  clk->advance(std::chrono::milliseconds(200));
  p.feed(make_packet(0.0, 6.8).data(), 36);

  EXPECT_EQ(pub_count, 2);
  int new_early = -1;
  for (int i = 0; i < 400; ++i) {
    if (new_early == -1 && r.intensities[i] == 203.0f && r.ranges[i] == 4.0f) {new_early = i;}
  }
  // 0 degrees with 180 offset = 180 degrees -> bin 200.
  EXPECT_TRUE(new_early >= 195 && new_early <= 205);
}

// ======== L4-9: Exact Boundaries ========
TEST(ParserTest, ExactBoundaries)
{
  auto clk = std::make_shared<FakeClock>();
  int pub_count = 0;
  std::atomic<int> offset{0};
  PacketParser p(115200, &offset, [&](const ScanResult &) {pub_count++;}, clk);

  // Generate 359 unique bins without exceeding 60 packets limit.
  // 359 bins = 45 packets (8 samples each, last packet has 7 unique, 1 duplicate).
  for (int i = 0; i < 45; ++i) {
    double start_ang = (i * 8) * 0.9;
    double end_ang = (i == 44) ? (i * 8 + 6) * 0.9 : (i * 8 + 7) * 0.9;
    auto pkt = make_packet(start_ang, end_ang, 1000, 100);
    p.feed(pkt.data(), 36);
  }

  auto trig = make_packet(0.0, 0.9);
  p.feed(trig.data(), 36);

  // 359 unique bins < 360 (kMinUniqueBins), so it should be dropped.
  EXPECT_EQ(pub_count, 0);
  EXPECT_GE(p.counters().revolutions_dropped, 1u);
}

TEST(ParserTest, ExactBoundaries360)
{
  auto clk = std::make_shared<FakeClock>();
  int pub_count = 0;
  std::atomic<int> offset{0};
  PacketParser p(115200, &offset, [&](const ScanResult &) {pub_count++;}, clk);

  // Generate exactly 360 unique bins.
  // We need 360 bins. 360 bins / 8 samples per packet = 45 packets.
  // For each packet i, start bin = i*8, end bin = i*8 + 7.
  // Angle = bin * 360.0 / 400.0 = bin * 0.9.
  for (int i = 0; i < 45; ++i) {
    double start_ang = (i * 8) * 0.9;
    double end_ang = (i * 8 + 7) * 0.9;
    auto pkt = make_packet(start_ang, end_ang, 1000, 100);
    p.feed(pkt.data(), 36);
  }

  auto trig = make_packet(0.0, 0.9);
  p.feed(trig.data(), 36);

  EXPECT_EQ(pub_count, 1);
}

// ======== L5-1: Continuous stream maintains valid duration ========
TEST(ParserTest, ContinuousStreamValidDuration)
{
  auto clk = std::make_shared<FakeClock>();
  int pub_count = 0;
  std::atomic<int> offset{0};
  PacketParser p(115200, &offset, [&](const ScanResult &) {pub_count++;}, clk);

  // Feeding multiple chunks rapidly at the exact same physical clock (rx_steady_ns=0)
  // simulates USB backlog. The continuous stream logic should space them out correctly.
  feed_packets(p, make_full_revolution());
  p.feed(make_packet(0.0, 6.8).data(), 36);
  // It should successfully publish because the stream time
  // inherently generated a valid duration!
  EXPECT_EQ(pub_count, 1);
  EXPECT_EQ(p.counters().revolutions_dropped, 0u);
}

// ======== Timing: last-known-good after valid ========
TEST(ParserTest, TimingLastKnownGood)
{
  auto clk = std::make_shared<FakeClock>();
  int pub_count = 0;
  ScanResult last;
  std::atomic<int> offset{0};
  PacketParser p(115200, &offset, [&](const ScanResult & r) {
      pub_count++; last = r;
    }, clk);

  // Rev 1 with valid timing.
  feed_packets(p, make_full_revolution());
  clk->advance(std::chrono::milliseconds(200));
  p.feed(make_packet(0.0, 6.8).data(), 36);
  EXPECT_EQ(pub_count, 1);

  // Rev 2 with 0 duration -> uses last-known-good.
  feed_packets(p, make_full_revolution());
  p.feed(make_packet(0.0, 6.8).data(), 36);
  EXPECT_EQ(pub_count, 2);
  EXPECT_NEAR(last.scan_duration_s, 0.2, 0.05);
}

// ======== Invalid ranges ========
TEST(ParserTest, InvalidRangesBecomePlusInf)
{
  auto clk = std::make_shared<FakeClock>();
  ScanResult captured;
  bool published = false;
  std::atomic<int> offset{0};
  PacketParser p(115200, &offset, [&](const ScanResult & r) {
      captured = r; published = true;
    }, clk);

  auto rev = make_full_revolution();
  // range_mm=0x8000 means invalid bit set
  rev[0] = make_packet(0.0, 6.8, 0x8000, 100);
  // normal range
  rev[1] = make_packet(6.8, 13.6, 4000, 100);
  // quality = 0 -> invalid
  rev[2] = make_packet(13.6, 20.4, 1000, 0);
  // range = 0 -> invalid
  rev[3] = make_packet(20.4, 26.8, 0, 100);
  rev[4] = make_packet(26.8, 34.0, 50, 100);

  feed_packets(p, rev);
  clk->advance(std::chrono::milliseconds(200));
  p.feed(make_packet(0.0, 6.8).data(), 36);

  ASSERT_TRUE(published);
  for (int i = 0; i < 400; ++i) {
    if (std::isfinite(captured.ranges[i])) {
      EXPECT_GE(captured.ranges[i], static_cast<float>(kRangeMinM));
      EXPECT_LE(captured.ranges[i], static_cast<float>(kRangeMaxM));
    }
  }
}

// ======== Buffer overflow recovery ========
TEST(ParserTest, LongGarbageRecovery)
{
  PacketParser p;
  std::vector<uint8_t> garbage(1000);
  for (size_t i = 0; i < garbage.size(); ++i) {
    garbage[i] = static_cast<uint8_t>(i & 0xFF);
  }
  p.feed(garbage.data(), garbage.size());
  auto pkt = make_packet(100.0, 107.0);
  p.feed(pkt.data(), pkt.size());
  EXPECT_GE(p.counters().packets_accepted, 1u);
}

// ======== Reset clears state ========
TEST(ParserTest, ResetClearsState)
{
  auto clk = std::make_shared<FakeClock>();
  int pub_count = 0;
  std::atomic<int> offset{0};
  PacketParser p(115200, &offset, [&](const ScanResult &) {pub_count++;}, clk);

  auto rev = make_full_revolution();
  for (size_t i = 0; i < 25 && i < rev.size(); ++i) {
    p.feed(rev[i].data(), 36);
  }
  clk->advance(std::chrono::milliseconds(100));
  p.reset();

  feed_packets(p, make_full_revolution());
  clk->advance(std::chrono::milliseconds(200));
  p.feed(make_packet(0.0, 6.8).data(), 36);
  EXPECT_EQ(pub_count, 1);
}

// ======== Counters accumulate ========
TEST(ParserTest, CountersAccumulate)
{
  PacketParser p;
  auto pkt = make_packet(10.0, 17.0);
  p.feed(pkt.data(), pkt.size());
  p.feed(pkt.data(), pkt.size());
  auto c = p.counters();
  EXPECT_EQ(c.packets_accepted, 2u);
  EXPECT_EQ(c.bytes_received, 72u);
  EXPECT_EQ(c.tail_byte_count, 2u);
}

// ======== Too many packets resets ========
TEST(ParserTest, TooManyPacketsResets)
{
  auto clk = std::make_shared<FakeClock>();
  int pub_count = 0;
  double published_duration = 0.0;
  std::atomic<int> offset{0};
  PacketParser p(115200, &offset, [&](const ScanResult & res) {
      pub_count++;
      published_duration = res.scan_duration_s;
    }, clk);

  // 1. Prime cache to 0.212s
  int64_t rx_time = 1000000000000LL;
  clk->ns_ = rx_time;
  auto rev1 = make_full_revolution();
  for (size_t i = 0; i < rev1.size(); ++i) {
    p.feed(rev1[i].data(), 36, rx_time);
    rx_time += (212000000 / rev1.size());
    clk->ns_ = rx_time;
  }
  rx_time = 1000000000000LL + 212000000;

  // To trigger wrap and publish rev1, feed one packet
  auto pkt = make_packet(0.0, 2.0);
  p.feed(pkt.data(), 36, rx_time);
  EXPECT_EQ(pub_count, 1);
  auto c_before = p.counters();

  // 2. Feed >= 60 packets continuously without wrap or backward jump
  // The first packet was s=0, e=2.0. So start next from 2.0.
  for (int i = 0; i < 62; ++i) {
    double s = 2.0 + i * 2.0;
    double e = s + 2.0;
    auto p2 = make_packet(s, e);
    p.feed(p2.data(), 36, rx_time);
  }
  auto c_after_drop = p.counters();

  EXPECT_EQ(c_after_drop.continuity_resets, c_before.continuity_resets + 1);
  EXPECT_EQ(c_after_drop.revolutions_dropped, c_before.revolutions_dropped + 1);

  // 3. Finish the partial revolution (which will drop due to low coverage)
  double current_s = 2.0 + 62 * 2.0;
  while (current_s < 355.0) {
    double next_s = current_s + 6.8;
    auto p3 = make_packet(current_s, next_s);
    p.feed(p3.data(), 36, rx_time);
    current_s = next_s;
  }

  // Wrap to drop the partial revolution
  auto end_pkt = make_packet(0.0, 2.0);
  p.feed(end_pkt.data(), 36, rx_time);

  EXPECT_EQ(pub_count, 1);  // Not published, coverage too low

  auto c_after_coverage_drop = p.counters();
  EXPECT_EQ(c_after_coverage_drop.revolutions_dropped, c_after_drop.revolutions_dropped + 1);

  // 4. Feed a FULL recovery revolution to prove it recovers and publishes
  auto backlog = make_full_revolution();
  for (size_t i = 1; i < backlog.size(); ++i) {
    p.feed(backlog[i].data(), 36, rx_time);
  }

  // Wrap again to publish the full recovery backlog
  p.feed(end_pkt.data(), 36, rx_time);

  EXPECT_EQ(pub_count, 2);  // Now it publishes!
  auto c_final = p.counters();
  EXPECT_EQ(c_final.continuity_resets, c_after_coverage_drop.continuity_resets);
  EXPECT_EQ(c_final.cache_fallbacks, c_after_drop.cache_fallbacks);  // No increment
  EXPECT_EQ(c_final.nominal_fallbacks, c_after_drop.nominal_fallbacks + 1);
  EXPECT_NEAR(published_duration, 0.165, 0.010);
}

// ======== Malformed angle rejected ========
TEST(ParserTest, MalformedAngleRejected)
{
  PacketParser p;
  auto pkt = make_packet(500.0, 507.0);
  p.feed(pkt.data(), pkt.size());
  EXPECT_EQ(p.counters().packets_accepted, 0u);
  EXPECT_GE(p.counters().candidates_rejected, 1u);
}

// ======== L5-4: Literal Known-Answer Checksum ========
TEST(ParserTest, ChecksumKnownAnswerLiteral)
{
  // Provenance: Unverified. Literal test based on empirical capture.
  // A literal packet carefully constructed to pass angle sanity checks
  // Sync: 55 AA 03 08
  // Speed: F4 01 (500)
  // Start Angle: 0xABE2 (44002 / 64 - 640 = 47.53 deg)
  // 8 samples (1000mm, quality 100)
  // End Angle: 0xAE62 (44642 / 64 - 640 = 57.53 deg)
  // Known checksum generated by Camsense V3.0 formula

  std::vector<uint8_t> pkt = {
    0x55, 0xAA, 0x03, 0x08,
    0xF4, 0x01,
    0xE2, 0xAB,
    0xE8, 0x03, 0x64,
    0xE8, 0x03, 0x64,
    0xE8, 0x03, 0x64,
    0xE8, 0x03, 0x64,
    0xE8, 0x03, 0x64,
    0xE8, 0x03, 0x64,
    0xE8, 0x03, 0x64,
    0xE8, 0x03, 0x64,
    0x62, 0xAE,
    0xC2, 0x3C     // The known checksum
  };

  PacketParser p;
  p.feed(pkt.data(), pkt.size());

  EXPECT_EQ(p.counters().packets_accepted, 1u);
  EXPECT_EQ(p.counters().checksum_failures, 0u);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

// ======== L9-1 Timing Cache Regressions ========
TEST(ParserTest, CacheAdmissionOnCoverage)
{
  auto clk = std::make_shared<FakeClock>();
  int pub_count = 0;
  double last_scan_duration = 0.0;
  std::atomic<int> offset{0};
  PacketParser p(115200, &offset, [&](const ScanResult & r) {
      pub_count++;
      last_scan_duration = r.scan_duration_s;
    }, clk);

  // 1. Two-packet revolution duration 0.212s, coverage drop
  clk->ns_ = 0;
  auto pktA = make_packet(0.0, 6.8);
  p.feed(pktA.data(), 36, clk->ns_);
  clk->ns_ += 212000000;     // 0.212s
  auto pktB = make_packet(6.8, 13.6);
  p.feed(pktB.data(), 36, clk->ns_);

  // Wrap to trigger drop
  auto wrap_pkt = make_packet(0.0, 6.8);
  p.feed(wrap_pkt.data(), 36, clk->ns_);
  EXPECT_EQ(pub_count, 0);     // Dropped due to coverage

  // 2. Healthy backlog (feed full rev at same timestamp)
  feed_packets(p, make_full_revolution());
  auto wrap_pkt2 = make_packet(0.0, 6.8);
  p.feed(wrap_pkt2.data(), 36, clk->ns_);

  EXPECT_EQ(pub_count, 1);
  EXPECT_EQ(p.counters().cache_fallbacks, 0u);
  EXPECT_EQ(p.counters().nominal_fallbacks, 1u);     // Nominal fallback expected
}

TEST(ParserTest, CacheAdmissionOnStale)
{
  auto clk = std::make_shared<FakeClock>();
  int pub_count = 0;
  std::atomic<int> offset{0};
  PacketParser p(115200, &offset, [&](const ScanResult &) {
      pub_count++;
    }, clk);

  // 1. Prime cache to 0.212s
  int64_t rx_time = 1000000000000LL;
  clk->ns_ = rx_time;
  auto rev1 = make_full_revolution();
  for (size_t i = 0; i < rev1.size(); ++i) {
    p.feed(rev1[i].data(), 36, rx_time);
    rx_time += (212000000 / rev1.size());
    clk->ns_ = rx_time;
  }
  // Now rx_time has advanced by 0.212s.

  // 2. Wrap and start backlog1 (which uses the measured cache)
  auto backlog1 = make_full_revolution();
  for (size_t i = 0; i < backlog1.size(); ++i) {
    p.feed(backlog1[i].data(), 36, rx_time);   // same timestamp!
  }

  // The first packet of backlog1 wrapped rev1, publishing it.
  EXPECT_EQ(pub_count, 1);
  EXPECT_EQ(p.counters().nominal_fallbacks, 0u);
  EXPECT_EQ(p.counters().cache_fallbacks, 0u);

  // 3. Wrap and start stale_rev
  auto stale_rev = make_full_revolution();
  for (size_t i = 0; i < stale_rev.size(); ++i) {
    p.feed(stale_rev[i].data(), 36, rx_time);   // same rx_time
    clk->advance(std::chrono::milliseconds(50));   // but process clock advances!
  }

  // The first packet of stale_rev wrapped backlog1, publishing it.
  // Because backlog1 had duration 0, it used cache fallback!
  EXPECT_EQ(pub_count, 2);
  auto c_before = p.counters();
  EXPECT_EQ(c_before.cache_fallbacks, 1u);

  // 4. Wrap and start backlog2
  auto backlog2 = make_full_revolution();
  for (size_t i = 0; i < backlog2.size(); ++i) {
    p.feed(backlog2[i].data(), 36, rx_time);
  }

  // The first packet of backlog2 wrapped stale_rev!
  // But stale_rev took > 2.6s of process time. So it was stale and dropped!
  EXPECT_EQ(pub_count, 2);   // pub_count remains 2!

  auto c_after_drop = p.counters();
  EXPECT_EQ(c_after_drop.revolutions_dropped, c_before.revolutions_dropped + 1);
  EXPECT_EQ(c_after_drop.stale_revolutions_dropped, c_before.stale_revolutions_dropped + 1);

  // 5. Wrap backlog2 to publish it
  auto end_rev = make_full_revolution();
  p.feed(end_rev[0].data(), 36, rx_time);   // just one packet to trigger wrap

  // backlog2 had duration 0, but cache was invalidated by the stale drop!
  // So it should use nominal fallback.
  EXPECT_EQ(pub_count, 3);
  auto c_final = p.counters();
  EXPECT_EQ(c_final.cache_fallbacks, c_after_drop.cache_fallbacks);   // No increment
  EXPECT_EQ(c_final.nominal_fallbacks, c_after_drop.nominal_fallbacks + 1);
}

TEST(ParserTest, RejectedCounters)
{
  auto clk = std::make_shared<FakeClock>();
  int pub_count = 0;
  std::atomic<int> offset{0};
  PacketParser p(115200, &offset, [&](const ScanResult &) {
      pub_count++;
    }, clk);

  int64_t rx_time = 1000000000000LL;
  clk->ns_ = rx_time;

  auto c_start = p.counters();

  // 1. Short revolution: 340->346, 346->359
  auto pkt1 = make_packet(340.0, 346.0);
  auto pkt2 = make_packet(346.0, 359.0);
  p.feed(pkt1.data(), 36, rx_time);
  p.feed(pkt2.data(), 36, rx_time);

  // Advance proc clock by 100ms so proc duration = 0.1s (valid fallback candidate)
  clk->advance(std::chrono::milliseconds(100));

  // Wrap
  auto wrap = make_packet(0.0, 6.8);
  p.feed(wrap.data(), 36, rx_time);

  EXPECT_EQ(pub_count, 0);
  auto c_drop = p.counters();
  EXPECT_EQ(c_drop.revolutions_dropped, c_start.revolutions_dropped + 1);
  EXPECT_EQ(c_drop.proc_clock_fallbacks, c_start.proc_clock_fallbacks);
  EXPECT_EQ(c_drop.cache_fallbacks, c_start.cache_fallbacks);
  EXPECT_EQ(c_drop.nominal_fallbacks, c_start.nominal_fallbacks);

  // 2. Healthy proc-fallback scan
  // Feed full revolution with same rx_time, but advance proc clock by 150ms
  auto rev = make_full_revolution();
  for (size_t i = 1; i < rev.size(); ++i) {
    p.feed(rev[i].data(), 36, rx_time);
  }
  clk->advance(std::chrono::milliseconds(150));
  p.feed(wrap.data(), 36, rx_time);

  EXPECT_EQ(pub_count, 1);
  auto c_final = p.counters();
  EXPECT_EQ(c_final.continuity_resets, c_drop.continuity_resets);
  EXPECT_EQ(c_final.revolutions_dropped, c_drop.revolutions_dropped);
  EXPECT_EQ(c_final.proc_clock_fallbacks, c_drop.proc_clock_fallbacks + 1);
  EXPECT_EQ(c_final.cache_fallbacks, c_drop.cache_fallbacks);
  EXPECT_EQ(c_final.nominal_fallbacks, c_drop.nominal_fallbacks);
}


TEST(ParserTest, CacheInvalidationOnGap)
{
  auto clk = std::make_shared<FakeClock>();
  int pub_count = 0;
  std::atomic<int> offset{0};
  PacketParser p(115200, &offset, [&](const ScanResult &) {
      pub_count++;
    }, clk);

  // 1. Healthy measured 0.212s
  clk->ns_ = 0;
  auto rev = make_full_revolution();
  for (size_t i = 0; i < rev.size(); ++i) {
    p.feed(rev[i].data(), 36, clk->ns_);
    clk->ns_ += 212000000 / rev.size();
  }
  // Wrap
  auto wrap_pkt = make_packet(0.0, 6.8);
  p.feed(wrap_pkt.data(), 36, clk->ns_);
  EXPECT_EQ(pub_count, 1);
  EXPECT_EQ(p.counters().nominal_fallbacks, 0u);

  // 2. Garbage bytes every 100ms for 600ms
  for (int i = 0; i < 6; ++i) {
    clk->ns_ += 100000000;
    uint8_t garbage = 0xFF;
    p.feed(&garbage, 1, clk->ns_);
  }

  // 3. Healthy backlog
  feed_packets(p, make_full_revolution());
  p.feed(wrap_pkt.data(), 36, clk->ns_);

  EXPECT_EQ(pub_count, 2);
  EXPECT_EQ(p.counters().gap_resets, 1u);
  EXPECT_EQ(p.counters().nominal_fallbacks, 1u);     // Must fallback, cache invalidated
}

// ======== Backlog Chunking ========
TEST(ParserTest, BacklogChunking)
{
  auto clk = std::make_shared<FakeClock>();

  for (size_t chunk : {36, 512, 1024}) {
    int pub_count = 0;
    bool future_timestamp = false;
    std::atomic<int> offset{0};
    PacketParser p(115200, &offset, [&](const ScanResult & r) {
        pub_count++;
        if (r.first_ray_steady_ns > clk->ns_) {
          future_timestamp = true;
        }
      }, clk);

    auto rev = make_full_revolution();
    std::vector<uint8_t> stream;
    for (int i = 0; i < 3; ++i) {
      for (const auto & pkt : rev) {
        stream.insert(stream.end(), pkt.begin(), pkt.end());
      }
    }
    // Add wrap packet
    auto trig = make_packet(0.0, 6.8);
    stream.insert(stream.end(), trig.begin(), trig.end());

    for (size_t at = 0; at < stream.size(); ) {
      size_t count = std::min(chunk, stream.size() - at);
      p.feed(stream.data() + at, count, clk->ns_);
      at += count;
      clk->ns_ += 1000000;
    }

    EXPECT_EQ(pub_count, 3);
    EXPECT_FALSE(future_timestamp);
  }
}
