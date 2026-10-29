// Copyright 2024 Codex
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

#pragma once
#include <vector>
#include <cstdint>

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
  uint16_t range_mm = 1000, uint8_t quality = 100)
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
