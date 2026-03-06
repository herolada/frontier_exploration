#ifndef ABSTRACT_LOGGER_H
#define ABSTRACT_LOGGER_H

#include <chrono>
#include <cstdint>
#include <format>
#include <string>
#include <unordered_map>

/**
 * @brief Abstract logger interface.
 *
 * Derive and implement log_impl() to plug in any concrete logging backend.
 * Throttled variants suppress repeated messages within a given time window,
 * keyed by a caller-supplied tag (use the LOG_*_THROTTLE macros for automatic
 * file:line tags).
 */
class AbstractLogger {
public:
  enum class Level : uint8_t { Debug = 0, Info = 1, Warn = 2, Error = 3 };

  virtual ~AbstractLogger() = default;

  // ---- Plain log methods --------------------------------------------------

  template <typename... Args>
  void debug(std::format_string<Args...> fmt, Args &&...args) {
    log_impl(Level::Debug, std::format(fmt, std::forward<Args>(args)...));
  }

  template <typename... Args>
  void info(std::format_string<Args...> fmt, Args &&...args) {
    log_impl(Level::Info, std::format(fmt, std::forward<Args>(args)...));
  }

  template <typename... Args>
  void warn(std::format_string<Args...> fmt, Args &&...args) {
    log_impl(Level::Warn, std::format(fmt, std::forward<Args>(args)...));
  }

  template <typename... Args>
  void error(std::format_string<Args...> fmt, Args &&...args) {
    log_impl(Level::Error, std::format(fmt, std::forward<Args>(args)...));
  }

  // ---- Throttled log methods ----------------------------------------------
  // `period_ms`  – minimum milliseconds between emissions for this tag
  // `tag`        – unique key per call-site; use the macros below

  template <typename... Args>
  void debug_throttle(uint32_t period_ms, const std::string &tag,
                      std::format_string<Args...> fmt, Args &&...args) {
    if (should_emit(tag, period_ms))
      log_impl(Level::Debug, std::format(fmt, std::forward<Args>(args)...));
  }

  template <typename... Args>
  void info_throttle(uint32_t period_ms, const std::string &tag,
                     std::format_string<Args...> fmt, Args &&...args) {
    if (should_emit(tag, period_ms))
      log_impl(Level::Info, std::format(fmt, std::forward<Args>(args)...));
  }

  template <typename... Args>
  void warn_throttle(uint32_t period_ms, const std::string &tag,
                     std::format_string<Args...> fmt, Args &&...args) {
    if (should_emit(tag, period_ms))
      log_impl(Level::Warn, std::format(fmt, std::forward<Args>(args)...));
  }

  template <typename... Args>
  void error_throttle(uint32_t period_ms, const std::string &tag,
                      std::format_string<Args...> fmt, Args &&...args) {
    if (should_emit(tag, period_ms))
      log_impl(Level::Error, std::format(fmt, std::forward<Args>(args)...));
  }

protected:
  /// Subclasses implement this one method; message is already formatted.
  virtual void log_impl(Level level, const std::string &message) = 0;

private:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  std::unordered_map<std::string, TimePoint> throttle_map_;

  bool should_emit(const std::string &tag, uint32_t period_ms) {
    auto now = Clock::now();
    auto it = throttle_map_.find(tag);
    if (it == throttle_map_.end() ||
        std::chrono::duration_cast<std::chrono::milliseconds>(now - it->second)
                .count() >= static_cast<int64_t>(period_ms)) {
      throttle_map_[tag] = now;
      return true;
    }
    return false;
  }
};

// Convenience macros – auto-generate a unique tag from the call-site location.
#define LOG_DEBUG_THROTTLE(logger, ms, ...)                                    \
  (logger).debug_throttle(                                                     \
      (ms), std::string(__FILE__) + ":" + std::to_string(__LINE__),            \
      __VA_ARGS__)
#define LOG_INFO_THROTTLE(logger, ms, ...)                                     \
  (logger).info_throttle(                                                      \
      (ms), std::string(__FILE__) + ":" + std::to_string(__LINE__),            \
      __VA_ARGS__)
#define LOG_WARN_THROTTLE(logger, ms, ...)                                     \
  (logger).warn_throttle(                                                      \
      (ms), std::string(__FILE__) + ":" + std::to_string(__LINE__),            \
      __VA_ARGS__)
#define LOG_ERROR_THROTTLE(logger, ms, ...)                                    \
  (logger).error_throttle(                                                     \
      (ms), std::string(__FILE__) + ":" + std::to_string(__LINE__),            \
      __VA_ARGS__)

#endif