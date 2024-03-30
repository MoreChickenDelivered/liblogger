//
// (C) 2019,2023 Elias Benali, TT.io
// Author: Elias Benali <ebenali@tradeterminal.io>
// Created by deb on 11/23/19.
//

#ifndef TH2_LOGUTIL_H
#define TH2_LOGUTIL_H

#include <bits/types/sigset_t.h>
#include <chalk/chalk.h>
#include <date/date.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <sys/epoll.h>
#include <sys/mman.h>
#include <sys/signalfd.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <boost/lockfree/policies.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <cfloat>
#include <chrono>
#include <cpptrace/cpptrace.hpp>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <exception>
#include <format>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <ranges>
#include <source_location>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>

#if (__cplusplus >= 202002L)
template <ssize_t N>
constexpr auto _basename(const char (&path)[N]) {
  ssize_t idx;
  for (idx = N - 1; idx >= 0; --idx)
    if (path[idx] == '/') break;
  return &(path[idx + 1]);
}
#else
inline const char *_basename(const char *path) {
  const char *p = path + strlen(path);
  while (p > path && *p != '/') --p;
  return p == path ? path : p + 1;
}
#endif

struct Logger;

namespace logutil {
namespace details {

template <typename... Args>
static inline auto fmt_args(auto &&fmt_str, Args &&...args) {
  return (std::vformat(std::forward<decltype(fmt_str)>(fmt_str),
                       std::make_format_args(std::forward<Args &>(args)...)));
}

template <typename... Args>
static inline auto fmt_args(const auto &fmt_str, Args &...args) {
  return (std::vformat(std::forward<decltype(fmt_str)>(fmt_str),
                       std::make_format_args(std::forward<Args &>(args)...)));
}

static constexpr auto kGetEnv = [](const char *env_var_name) {
  if (auto *env_var = getenv(env_var_name); env_var != nullptr) {
    auto sv_env = std::string_view{env_var};
    if (sv_env == "0" || sv_env == "n" || sv_env == "N") return false;
    if (auto itr =
            std::ranges::count(std::vector({"0", "n", "N", "no", "NO", "false",
                                            "FALSE", "off", "OFF"}),
                               sv_env);
        itr)
      return false;
    return true;
  }
  return false;
};

namespace explode {
template <uint8_t... Digits>
struct PositiveToChars {
  static const std::array<char, sizeof...(Digits) + 1> kValue;
};
template <uint8_t... Digits>
constexpr std::array<char, sizeof...(Digits) + 1>
    PositiveToChars<Digits...>::kValue = {('0' + Digits)..., 0};

template <uint8_t... Digits>
struct NegativeToChars {
  static const std::array<char, sizeof...(Digits) + 2> kValue;
};
template <uint8_t... Digits>
constexpr std::array<char, sizeof...(Digits) + 2>
    NegativeToChars<Digits...>::kValue = {'-', ('0' + Digits)..., 0};

template <bool Neg, uint8_t... Digits>
struct ToChars : PositiveToChars<Digits...> {};

template <uint8_t... Digits>
struct ToChars<true, Digits...> : NegativeToChars<Digits...> {};

template <bool Neg, uintmax_t Rem, uint8_t... Digits>
struct Explode : Explode<Neg, Rem / 10, Rem % 10, Digits...> {};

template <bool Neg, uint8_t... Digits>
struct Explode<Neg, 0, Digits...> : ToChars<Neg, Digits...> {};

template <typename T>
constexpr uintmax_t cabs(T num) {
  return (num < 0) ? -num : num;
}
}  // namespace explode

template <typename Integer, Integer Num>
struct StringFrom : explode::Explode<(Num < 0), explode::cabs(Num)> {};

/// https://stackoverflow.com/a/68911826
template <size_t N>
struct PastLastSlash {
  consteval PastLastSlash(const char (&src)[N]) {
    std::string str{src};
    auto pos = str.rfind('/');
    if (pos == std::string::npos) {
      pos = 0;
    } else {
      ++pos;
    }
    size_t len = str.length() - pos;
    std::copy(src + pos, src + pos + len - 1, value.data());
  }

  [[nodiscard]] consteval size_t len() const {
    std::string str{value.data()};
    return str.length();
  }

  std::array<char, N - 1> value{};
};

template <ssize_t N>
consteval std::string basename(const char (&path)[N]) {
  ssize_t idx;
  for (idx = N - 1; idx >= 0; --idx)
    if (path[idx] == '/') break;
  return (path + idx + 1);
}

template <typename... Args>
static inline void crash_with_info(std::source_location sloc,
                                   std::format_string<Args...> fmt_str,
                                   Args &&...args) {
  throw std::runtime_error{
      std::format("\n{}:{}:{}: {}", ::basename(sloc.file_name()), sloc.line(),
                  sloc.function_name(),
                  chalk::fg::ANSI8<std::numeric_limits<uint8_t>::max(), 0, 0>(
                      std::format(fmt_str, std::forward<Args>(args)...)))};
}

static inline auto tabstops(std::string str, off_t ntabs) {
  auto ptr = str.begin();
  while (ptr != str.end()) {
    ptr = (ptr == str.begin() || *(ptr - 1) == '\n')
              ? str.insert(ptr, ntabs, '\t') + ntabs
              : ptr + 1;
  }
  return str;
}
}  // namespace details

template <details::PastLastSlash S>
struct PastLastSlash {
  static consteval std::array<char, S.len() + 1> copy() {
    std::array<char, S.len() + 1> arr{};
    std::copy(S.value.data(), S.value.data() + S.len(), arr.data());
    return arr;
  }
  static constexpr std::array<char, S.len() + 1> kArr{copy()};
};

template <std::size_t... Zs>
constexpr std::string zstrcats(const std::array<char, Zs> &...strN)
  requires(noexcept(strN[0]) && ...)
{
  std::array<char, (Zs + ...) - sizeof...(Zs) + 1> resv{};
  size_t cat_idx = 0;
  const auto cat_helper = [&](const auto &src) {
    for (auto chr : src) resv[cat_idx++] = chr;
    if (!src.empty()) cat_idx -= 1;
  };

  (void)(cat_helper(strN), ...);

  return std::string{std::string_view(resv.data())};
}

template <typename... StrViews>
constexpr std::string zstrcats(StrViews... strN) {
  std::string zsum;

  auto all_of_em = std::array<std::string, sizeof...(StrViews)>(
      {std::string{strN.data()}...});

  for (const auto &zstr : all_of_em) zsum += std::move(zstr);

  return zsum;
}

template <size_t M>
consteval std::array<char, M> zatoa(const char (&str1)[M])
  requires(noexcept(str1[0]))
{
  std::array<char, M> resv{};
  size_t conv_idx = 0;

  for (auto chr : std::span(str1, M - 1)) resv[conv_idx++] = chr;
  return resv;
}

template <size_t M>
consteval std::array<char, M> zatoa(const std::array<char, M> &str1) {
  std::array<char, M> resv{};
  size_t conv_idx = 0;

  for (auto chr : str1) resv[conv_idx++] = chr;
  return resv;
}

// static constexpr auto zatoa(std::string str1) { return std::move(str1); }
constexpr auto zatoa(std::string_view str1) {
  // return str1;
  auto new_str = std::string{str1.begin(), str1.end()};
  return std::move(new_str);
}

template <typename Clock = std::chrono::system_clock>
static inline auto isoDate(std::chrono::time_point<Clock> tpt = Clock::now())
    -> std::string {
  return date::format("%FT%TZ",
                      std::chrono::floor<std::chrono::milliseconds>(tpt));
}

static Logger &get() noexcept;

}  // namespace logutil

struct Logger {
  explicit Logger(int ost = STDOUT_FILENO, int est = STDERR_FILENO)
      : outStream_{ost}, errStream_{est} {}

  ~Logger() { flush(); }

  inline static void flush() {
    if (Logger::async_output_queue_) {
      std::tuple<int, std::function<std::string(std::string_view)>, std::string>
          output_elm;

      while (Logger::async_output_queue_->pop(output_elm)) {
        auto [ost, fmt, str] = std::move(output_elm);
        dprintf(ost, "%s", fmt && isatty(ost) ? fmt(str).c_str() : str.c_str());
      }
    }
  }

  template <typename... Args>
  inline auto Info(Args &&...args) {
    if (verbosity_ >= Verbosity::kInfo)
      output(std::chrono::system_clock::now(), outStream_, kInfoChalk,
             std::forward<Args>(args)...);
  }

  template <typename... Args>
  inline auto Warn(Args &&...args) {
    if (verbosity_ >= Verbosity::kWarn)
      output(std::chrono::system_clock::now(), outStream_, kWarnChalk,
             std::forward<Args>(args)...);
  }

  template <typename... Args>
  inline auto Error(Args &&...args) {
    if (verbosity_ >= Verbosity::kErr)
      output(std::chrono::system_clock::now(), errStream_, kErrorChalk,
             std::forward<Args>(args)...);
  }

  template <typename... Args>
  inline auto Debug(Args &&...args) {
    if (verbosity_ >= Verbosity::kDebug)
      output(std::chrono::system_clock::now(), errStream_, kDebugChalk,
             std::forward<Args>(args)...);
  }

#if !defined(NDEBUG) | defined(OVERRIDE_NDEBUG)
  template <typename... Args>
  inline auto Trace(Args &&...args) {
    if (verbosity_ >= Verbosity::kTrace)
      output(std::chrono::system_clock::now(), errStream_, kTraceChalk,
             std::forward<Args>(args)...);
  }
#else
  template <typename... Args>
  inline auto Trace(Args &&...args) {}
#endif

  void setUnifiedOutput(bool flag) { this->unified_output_ = flag; }

  enum class Verbosity : unsigned {
    kQuiet = 0,
    kErr,
    kWarn,
    kInfo,
    kDebug,
    kTrace,
  };

  void setVerbosity(Verbosity verbosity) { verbosity_ = verbosity; }

 protected:
  template <typename Clock, typename FmtType, typename... Args>
  auto output(std::chrono::time_point<Clock> &&tpt, int ost, FmtType &&fmt,
              auto &&fmt_str, Args &&...args) {
    const std::chrono::duration<float> delta_t =
        tpt - ((ost == outStream_ && !this->unified_output_) ? lastTimes_[0]
                                                             : lastTimes_[1]);

    lastTimes_[(ost == outStream_ && !this->unified_output_) ? 0 : 1] = tpt;

    async_output_queue_->push(std::make_tuple(
        ost, kTsChalk,
        logutil::isoDate(tpt) + "  " +
            fmt(logutil::details::fmt_args(
                std::forward<decltype(fmt_str)>(fmt_str),
                std::forward<Args>(args)...)) +
            " +" + kDtChalk(std::to_string(delta_t.count()) + "s") + "\n"));
  }

 private:
  int outStream_;
  int errStream_;
  std::chrono::time_point<std::chrono::system_clock> lastTimes_[2]{
      std::chrono::system_clock::now(), std::chrono::system_clock::now()};

  static constexpr auto kAsyncOutputQueueCapacity = 1024;
  static inline boost::lockfree::spsc_queue<
      std::tuple<int, std::function<std::string(std::string_view)>,
                 std::string>,
      boost::lockfree::capacity<kAsyncOutputQueueCapacity>>
      *async_output_queue_ = nullptr;

  bool unified_output_ = false;
  Verbosity verbosity_ = Verbosity::kDebug;

  static constexpr auto kTsChalk =
      chalk::compose(chalk::fg::Green, chalk::fmt::Bold);
  static constexpr auto kDtChalk =
      chalk::compose(chalk::fg::BrightCyan, chalk::fmt::Bold);
  static constexpr auto kInfoChalk = chalk::fg::White;
  static constexpr auto kErrorChalk = chalk::fg::Red;
  static constexpr auto kWarnChalk = chalk::fg::Yellow;
  static constexpr auto kDebugChalk = chalk::fg::Magenta;
  static constexpr auto kTraceChalk =
      chalk::compose(chalk::compose(chalk::BackgroundColor{chalk::bg::White},
                                    chalk::fg::ANSI8<45, 0, 255>),
                     chalk::fmt::Bold);

 public:
  /**
   * @brief Returns a reference to the Logger singleton.
   *
   * This function establishes a shared memory space to ensure multiple
   * processes share a single Logger instance. It's particularly useful when
   * separate shared objects, each containing a duplicate Logger instance due to
   * being compiled with the same header, need a unified logging mechanism. This
   * scenario often arises with runtime-linked components via `dlopen`.
   *
   * Process:
   * 1. Check for an existing shared memory area for the Logger. If it exists,
   *    map it into the current process's address space, retrieve the Logger
   *    instance, and return it.
   * 2. If no shared memory exists, create it and size it appropriately to store
   *    the Logger instance.
   * 3. Construct a new Logger in the shared memory.
   * 4. Once the Logger is stored, close and unmap the shared memory.
   * 5. Return the Logger instance.
   *
   * Errors during any step, such as failure to map or unmap the shared memory,
   * trigger a runtime exception.
   *
   * @return A reference to the Logger singleton.
   * @throws std::runtime_error if an error occurs during the process.
   */
  static inline auto &get() noexcept { return logutil::get(); }
  friend Logger &logutil::get() noexcept;
};

// #define CONCAT(_a_, _b_) CONCAT_INNER(_a_, _b_)
// #define CONCAT_INNER(_a_, _b_) _a_##_b_
// #define UNIQUE_NAME(_base_) \
//   CONCAT(CONCAT(CONCAT(_base_, __COUNTER__), _), __LINE__)

#define QT(x) #x
#define QUOTE(x) QT(x)

// #include <type_traits>
// #define IS_SL(x)                                                    \
//   ([&]<class T = char>() {                                          \
//     return std::is_same_v<decltype(x), T const(&)[sizeof(x)]> and   \
//            requires { std::type_identity_t<T[sizeof(x) + 1]>{x}; }; \
//   }())

#define INFO(_fmt, ...)                                                     \
  do {                                                                      \
    logger.Info(logutil::zstrcats(                                          \
                    logutil::zatoa("[II] ["),                               \
                    logutil::zatoa(logutil::PastLastSlash<__FILE__>::kArr), \
                    logutil::zatoa(":" QUOTE(__LINE__) ":"),                \
                    logutil::zatoa(__FUNCTION__), logutil::zatoa("]: "),    \
                    logutil::zatoa(_fmt))                                   \
                                                                            \
                    .data() __VA_OPT__(, ) __VA_ARGS__);                    \
  } while (false)

#define ERROR(_fmt, ...)                                                     \
  do {                                                                       \
    logger.Error(logutil::zstrcats(                                          \
                     logutil::zatoa("[EE] ["),                               \
                     logutil::zatoa(logutil::PastLastSlash<__FILE__>::kArr), \
                     logutil::zatoa(":" QUOTE(__LINE__) ":"),                \
                     logutil::zatoa(__FUNCTION__), logutil::zatoa("]: "),    \
                     logutil::zatoa(_fmt))                                   \
                                                                             \
                     .data() __VA_OPT__(, ) __VA_ARGS__);                    \
  } while (false)

#define WARN(_fmt, ...)                                                     \
  do {                                                                      \
    logger.Warn(logutil::zstrcats(                                          \
                    logutil::zatoa("[WW] ["),                               \
                    logutil::zatoa(logutil::PastLastSlash<__FILE__>::kArr), \
                    logutil::zatoa(":" QUOTE(__LINE__) ":"),                \
                    logutil::zatoa(__FUNCTION__), logutil::zatoa("]: "),    \
                    logutil::zatoa(_fmt))                                   \
                                                                            \
                    .data() __VA_OPT__(, ) __VA_ARGS__);                    \
  } while (false)

#define DEBUG(_fmt, ...)                                                     \
  do {                                                                       \
    logger.Debug(logutil::zstrcats(                                          \
                     logutil::zatoa("[DD] ["),                               \
                     logutil::zatoa(logutil::PastLastSlash<__FILE__>::kArr), \
                     logutil::zatoa(":" QUOTE(__LINE__) ":"),                \
                     logutil::zatoa(__FUNCTION__), logutil::zatoa("]: "),    \
                     logutil::zatoa(_fmt))                                   \
                                                                             \
                     .data() __VA_OPT__(, ) __VA_ARGS__);                    \
  } while (false)

#if defined(NDEBUG) && !defined(OVERRIDE_NDEBUG)
#define TRACE(...) \
  { ; }
#else
#define TRACE(_fmt, ...)                                                     \
  do {                                                                       \
    logger.Trace(logutil::zstrcats(                                          \
                     logutil::zatoa("[TT] ["),                               \
                     logutil::zatoa(logutil::PastLastSlash<__FILE__>::kArr), \
                     logutil::zatoa(":" QUOTE(__LINE__) ":"),                \
                     logutil::zatoa(__FUNCTION__), logutil::zatoa("]: "),    \
                     logutil::zatoa(_fmt))                                   \
                                                                             \
                     .data() __VA_OPT__(, ) __VA_ARGS__);                    \
  } while (false)

#endif

#endif
#ifndef RELEASE_ASSERT
#define RELEASE_ASSERT(assertion, _fmt, ...)                              \
  do {                                                                    \
    if (BOOST_UNLIKELY(!(assertion)))                                     \
      ([&](std::source_location sloc = std::source_location::current()) { \
        __atomic_thread_fence(__ATOMIC_SEQ_CST);                          \
        __atomic_signal_fence(__ATOMIC_SEQ_CST);                          \
        logutil::details::crash_with_info(                                \
            sloc,                                                         \
            "\nRuntime assertion failed:"                                 \
            "\n\tLocation: " __FILE__                                     \
            ":" QUOTE(__LINE__) "\n\tFailed assertion: " #assertion       \
                                "\n\tContext:\n{}",                       \
            logutil::details::tabstops(                                   \
                std::format(_fmt __VA_OPT__(, ) __VA_ARGS__), 2));        \
      }());                                                               \
  } while (false);

namespace logutil {
auto static get() noexcept -> Logger & {
  using ::basename;

  try {
    static const auto kSingleton = []() {
      const auto do_debug = logutil::details::kGetEnv("LOGUTIL_DEBUG");
      const auto is_realtime = logutil::details::kGetEnv("LOGUTIL_REALTIME");

      auto my_pid = getpid();
      constexpr auto kOffLogger = 0UL;
      constexpr auto kSzLogger = sizeof(std::shared_ptr<Logger>);
      constexpr auto kOffQueue = kOffLogger + kSzLogger;
      constexpr auto kSzQueue = sizeof(decltype(Logger::async_output_queue_));
      constexpr auto kSzCombined = kSzLogger + kSzQueue;

      if (const int shm_fd = shm_open(std::format("/{}-logger", my_pid).c_str(),
                                      O_RDWR, S_IRUSR | S_IWUSR);
          shm_fd != -1) {
        auto mmapped_shm = reinterpret_cast<uintptr_t>(
            mmap(nullptr, kSzCombined, PROT_READ | PROT_WRITE, MAP_SHARED,
                 shm_fd, 0));

        if (mmapped_shm == reinterpret_cast<uintptr_t>(MAP_FAILED))
          throw std::runtime_error{
              std::format("{}:{}:{}: on-restore mmap of shm area failed: {}",
                          basename(__FILE__), __LINE__, __PRETTY_FUNCTION__,
                          strerror(errno))};

        close(shm_fd);

        auto prev_logger = *reinterpret_cast<std::shared_ptr<Logger> *>(
            mmapped_shm + kOffLogger);

        auto *prev_queue =
            *reinterpret_cast<decltype(Logger::async_output_queue_) *>(
                mmapped_shm + kOffQueue);

        if (!prev_logger || !prev_queue)
          throw std::runtime_error{std::format(
              "{}:{}:{}: on-restore: NULL in the mmapped values (raw "
              "pointers: {:p},{:p})",
              basename(__FILE__), __LINE__, __PRETTY_FUNCTION__,
              static_pointer_cast<void>(prev_logger).get(),
              static_cast<void *>(prev_queue))};

        Logger::async_output_queue_ = prev_queue;

        if (do_debug) {
          dprintf(STDERR_FILENO,
                  "%s:%i:%s: restored logger instance form mmapped shared "
                  "memory at (%p,%p)\n",
                  basename(__FILE__), __LINE__, __PRETTY_FUNCTION__,
                  static_pointer_cast<void>(prev_logger).get(),
                  static_cast<void *>(prev_queue));
        }

        munmap(reinterpret_cast<void *>(mmapped_shm), kSzCombined);
        return prev_logger;
      }

      if (const int shm_fd =
              shm_open(std::format("/{}-logger", my_pid).c_str(),
                       O_CREAT | O_EXCL | O_RDWR, S_IRUSR | S_IWUSR);
          shm_fd != -1) {
        if (ftruncate(shm_fd, kSzCombined) == -1) {
          throw std::runtime_error{
              std::format("{}:{}:{}: ftruncate failed: {}", basename(__FILE__),
                          __LINE__, __PRETTY_FUNCTION__, strerror(errno))};
        }
        auto new_logger = std::make_shared<Logger>();
        auto *new_queue =
            new (std::pointer_traits<
                 decltype(Logger::async_output_queue_)>::element_type)();

        auto *mmapped_shm = static_cast<void *>(mmap(nullptr, kSzCombined,
                                                     PROT_READ | PROT_WRITE,
                                                     MAP_SHARED, shm_fd, 0));

        if (mmapped_shm == MAP_FAILED)
          throw std::runtime_error{
              std::format("{}:{}:{}: on-create mmap of shm area failed: {}",
                          basename(__FILE__), __LINE__, __PRETTY_FUNCTION__,
                          strerror(errno))};

        *reinterpret_cast<std::shared_ptr<Logger> *>(
            static_cast<char *>(mmapped_shm) + kOffLogger) = new_logger;

        *reinterpret_cast<decltype(Logger::async_output_queue_) *>(
            static_cast<char *>(mmapped_shm) + kOffQueue) = new_queue;

        close(shm_fd);
        munmap(mmapped_shm, kSzCombined);

        std::thread{[=]() {
          if (!is_realtime) {
            const auto my_pthread = pthread_self();

            const auto my_scheduler = sched_getscheduler(getpid());
            RELEASE_ASSERT(my_scheduler != -1, "sched_getscheduler failed: {}",
                           strerror(errno));

            const auto min_priority = sched_get_priority_min(my_scheduler);
            RELEASE_ASSERT(min_priority != -1,
                           "sched_get_priority_min failed: {}",
                           strerror(errno));

            RELEASE_ASSERT(
                pthread_setschedprio(pthread_self(), min_priority) == 0, "{}",
                strerror(errno));
          }

          if (do_debug)
            dprintf(STDERR_FILENO,
                    "%s:%i:%s: spawned logger output async queue thread... "
                    "(q-size=%zu, "
                    "realtime=%i)\n",
                    basename(__FILE__), __LINE__, __PRETTY_FUNCTION__,
                    sizeof(*Logger::async_output_queue_),
                    static_cast<int>(is_realtime));

          if (Logger::async_output_queue_ == nullptr) {
            if (do_debug)
              dprintf(STDERR_FILENO,
                      "%s:%i:%s: awaiting asychronous IPC state...\n",
                      basename(__FILE__), __LINE__, __PRETTY_FUNCTION__);
            while (!Logger::async_output_queue_);
            if (do_debug)
              dprintf(STDERR_FILENO, "%s:%i:%s: asychronous IPC state: set\n",
                      basename(__FILE__), __LINE__, __PRETTY_FUNCTION__);
          }

          // setup abnormal termination intercept via signalfd
          sigset_t mask;
          sigemptyset(&mask);
          sigaddset(&mask, SIGTERM);
          sigaddset(&mask, SIGINT);
          sigaddset(&mask, SIGQUIT);
          sigaddset(&mask, SIGKILL);
          sigaddset(&mask, SIGSEGV);
          sigaddset(&mask, SIGABRT);
          sigaddset(&mask, SIGBUS);
          sigaddset(&mask, SIGFPE);
          sigaddset(&mask, SIGILL);
          sigaddset(&mask, SIGSYS);

          if (sigprocmask(SIG_BLOCK, &mask, nullptr) == -1)
            throw std::runtime_error{std::format(
                "{}:{}:{}: sigprocmask failed: {}", basename(__FILE__),
                __LINE__, __PRETTY_FUNCTION__, strerror(errno))};

          auto sigfd = signalfd(-1, &mask, SFD_NONBLOCK | SFD_CLOEXEC);

          auto constexpr kInitBackoff = 1.0e3F;
          auto constexpr kBackoffGrowth = 1.1F;
          auto constexpr kBackoffIterLimit = 1e7F;
          constexpr auto kSleepDurationMillis = 100;

          auto constexpr kCheckSig = [&](int sigfd) {
            signalfd_siginfo snfo;
            ssize_t nread = read(sigfd, &snfo, sizeof(snfo));
            if (nread == -1) {
              if (errno == EAGAIN) return;
              throw std::runtime_error{
                  std::format("{}:{}:{}: read failed: {}", basename(__FILE__),
                              __LINE__, __PRETTY_FUNCTION__, strerror(errno))};
            }
            if (nread != sizeof(snfo))
              throw std::runtime_error{std::format(
                  "{}:{}:{}: read failed: short read", basename(__FILE__),
                  __LINE__, __PRETTY_FUNCTION__)};
            if (snfo.ssi_signo == SIGQUIT) {
              Logger::async_output_queue_->push(std::make_tuple(
                  STDERR_FILENO, Logger::kWarnChalk,
                  "received SIGQUIT, triggering graceful exit...\n"));
              Logger::flush();
              // exit(EXIT_SUCCESS);
            } else {
              Logger::async_output_queue_->push(std::make_tuple(
                  STDERR_FILENO, compose(Logger::kErrorChalk, chalk::fmt::Bold),
                  "received signal, triggering graceful exit...\n"));
              Logger::flush();
              // exit(EXIT_FAILURE);
            }
          };

          do {
            kCheckSig(sigfd);
            if (Logger::async_output_queue_->read_available()) {
              Logger::flush();
            } else {
              if (is_realtime) {
                size_t iters{};
                float backoff = kInitBackoff;
                while (!Logger::async_output_queue_->read_available()) {
                  if (iters >= static_cast<size_t>(kBackoffIterLimit)) {
                    std::this_thread::sleep_for(
                        std::chrono::duration<float, std::milli>{
                            sqrtf(kSleepDurationMillis)});
                    break;
                  }
                  if (iters++ > static_cast<size_t>(backoff)) {
                    backoff = powf(backoff, kBackoffGrowth);
                    std::this_thread::yield();
                  }
                }
              } else {
                while (!Logger::async_output_queue_->read_available()) {
                  std::this_thread::sleep_for(
                      std::chrono::milliseconds(kSleepDurationMillis));
                }
              }
            }
          } while (true);
        }}.detach();

        atexit([] {
          auto &logger = Logger::get();
          logger.~Logger();
        });

        if (do_debug) {
          dprintf(STDERR_FILENO, "%s:%i:%s: logger initialized (%p,%p)\n",
                  basename(__FILE__), __LINE__, __PRETTY_FUNCTION__,
                  new_logger.get(), new_queue);
        }

        Logger::async_output_queue_ = new_queue;
        return new_logger;
      }

      throw std::runtime_error{
          std::format("{}:{}:{}: unable to restore or create a new 'shm' "
                      "handle for Logger",
                      basename(__FILE__), __LINE__, __PRETTY_FUNCTION__)};
    }();
    return *kSingleton;
  } catch (std::exception const &err) {
    std::cerr << std::format(
        "{}:{}:{}: caught exception while constructing Logger singleton: "
        "{}\n",
        basename(__FILE__), __LINE__, __func__, err.what());
    std::terminate();
  }
}
}  // namespace logutil

#endif  // TH2_LOGUTIL_H
