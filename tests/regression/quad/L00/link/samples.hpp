// The sample messages of the link conformance suite: one per message in link/scope.txt plus a zero-tail case, with values in
// every field that the other language's suite (gcs/src/lib/link, tests/regression/quad/L00/link/ts) repeats exactly. A change
// here is a change there and in vectors/.
#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "marv/link/codec.hpp"

namespace marv::link::test {

struct Sample {
  std::string name;
  Message message;
};

template <std::size_t N>
std::array<char, N> text(std::string_view s) {
  std::array<char, N> out{};
  std::copy_n(s.begin(), std::min(s.size(), N), out.begin());
  return out;
}

template <std::size_t N>
std::array<std::uint8_t, N> ramp(std::uint8_t first) {
  std::array<std::uint8_t, N> out{};
  for (std::size_t i = 0; i < N; ++i) {
    out[i] = static_cast<std::uint8_t>(first + i);
  }
  return out;
}

inline std::vector<Sample> samples() {
  std::vector<Sample> out;
  out.push_back({"Heartbeat", Heartbeat{2, 8, 81, 0x01020304U, 4}});
  out.push_back({"AutopilotVersion",
                 AutopilotVersion{0x0123456789ABCDEFULL, 0x01020300U, 0x04050600U, 0x07080900U, 0x0A0B0C0DU, ramp<8>(1),
                                  ramp<8>(20), ramp<8>(40), 0x1234, 0x5678, 0xFEDCBA9876543210ULL, ramp<18>(100)}});
  out.push_back({"ParamRequestList", ParamRequestList{1, 2}});
  out.push_back({"ParamRequestListZero", ParamRequestList{0, 0}});
  out.push_back({"ParamRequestRead", ParamRequestRead{1, 2, text<16>("rate_kp_roll"), -1}});
  out.push_back({"ParamSet", ParamSet{1, 2, text<16>("rotor_posit_ab12"), 0.25F, 9}});
  out.push_back({"ParamValue", ParamValue{text<16>("mass_kg"), -1.5F, 6, 136, 17}});
  out.push_back({"Statustext", Statustext{4, text<50>("config refused: schema"), 5, 1}});
  out.push_back({"CommandLong", CommandLong{1, 2, 512, 1, 148.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, -1.5F}});
  out.push_back({"CommandAck", CommandAck{512, 3, 255, -7, 255, 190}});
  out.push_back({"MarvIdentity",
                 MarvIdentity{0x2FF36D1DA5FC3596ULL, ramp<32>(1), ramp<32>(50), ramp<32>(100), MarvConfigSource::Active,
                              MarvConfigState::Refused, MarvConfigRefusal::ReportBinding, MarvActivationResult::RefusedAtBoot,
                              1}});
  out.push_back({"MarvParamRecord",
                 MarvParamRecord{0x3F800000U, 0xC0200000U, 0.125F, 17, 136, 20, MarvParamOrigin::Manual,
                                 MarvParamMethod::Datasheet, MarvParamSigmaKind::Known, 1, 20,
                                 text<64>("UZH NeuroBEM Table 3")}});
  out.push_back({"MarvParamSource",
                 MarvParamSource{0x2FF36D1DA5FC3596ULL, 17, 64, 200, 1, 2, MarvParamSourceOp::Set, 64,
                                 text<64>("0123456789012345678901234567890123456789012345678901234567890123")}});
  out.push_back({"MarvConfigCommand",
                 MarvConfigCommand{0x2FF36D1DA5FC3596ULL, ramp<32>(3), ramp<32>(70), 1, 2, MarvConfigAction::Activate}});
  out.push_back({"MarvConfigAck", MarvConfigAck{255, 190, MarvConfigAction::ResetFactory, MarvConfigResult::DeniedArmed}});
  return out;
}

inline Envelope envelope_for(std::size_t index) {
  return {static_cast<std::uint8_t>(10 + index), 190, static_cast<std::uint8_t>(index)};
}

}  // namespace marv::link::test
