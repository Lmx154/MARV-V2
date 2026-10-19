// The logger's configuration from a parameter table (the log_* entries of fw/log/params/log_register.yaml, the design-budget
// entry sd_log_write_stall_max_s, and the tick rational and rate-loop divisor of the scenario register). Header-only and
// templated on nothing: include it in a translation unit that links one parameter set that carries those entries
// (marv_params_fc3v2_log); fw/log itself does not link a parameter set, so it stays usable with any.
#pragma once

#include <cstddef>
#include <cstdint>

#include <marv/params/param.hpp>
#include <marv/params/param_ids.hpp>

#include "marv/log/budget.hpp"
#include "marv/log/logger.hpp"

namespace marv::log {

[[nodiscard]] inline TimeUs seconds_to_us(float s) noexcept {
  return s > 0.0F ? static_cast<TimeUs>(static_cast<double>(s) * prim::kMicrosecondsPerSecond) : TimeUs{0};
}

[[nodiscard]] inline FastRates fast_rates_from_params(const ParamTable& t) noexcept {
  return fast_rates(static_cast<std::uint32_t>(param_value<ParamId::tick_period_num_us>(t)),
                    static_cast<std::uint32_t>(param_value<ParamId::tick_period_den>(t)),
                    static_cast<std::uint32_t>(param_value<ParamId::rate_loop_divisor>(t)),
                    static_cast<double>(param_value<ParamId::log_manual_rate_hz>(t)));
}

[[nodiscard]] inline double write_stall_max_s(const ParamTable& t) noexcept {
  return static_cast<double>(param_value<ParamId::sd_log_write_stall_max_s>(t));
}

// `ring_cells` is the ring's capacity; the pre-roll is derived from it (budget.hpp pre_roll_cells).
[[nodiscard]] inline LoggerConfig logger_config_from_params(const ParamTable& t, std::size_t ring_cells,
                                                            std::uint64_t (*session_id)(void*) noexcept,
                                                            void* session_ctx) noexcept {
  LoggerConfig c;
  c.file_bytes = static_cast<std::uint64_t>(param_value<ParamId::log_file_bytes>(t));
  c.post_roll_us = seconds_to_us(param_value<ParamId::log_post_roll_s>(t));
  c.status_period_us = seconds_to_us(param_value<ParamId::log_status_period_s>(t));
  c.slow_period_us = seconds_to_us(param_value<ParamId::log_slow_period_s>(t));
  c.reprobe_period_us = seconds_to_us(param_value<ParamId::log_reprobe_period_s>(t));
  c.pre_roll_cells = pre_roll_cells(ring_cells, fast_rates_from_params(t), write_stall_max_s(t));
  c.session_id = session_id;
  c.session_ctx = session_ctx;
  return c;
}

}  // namespace marv::log
