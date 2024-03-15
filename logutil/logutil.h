//
// (C) 2019,2023 Elias Benali, TT.io
// Author: Elias Benali <ebenali@tradeterminal.io>
// Created by deb on 11/23/19.
//

#ifndef TH2_LOGUTIL_H
#define TH2_LOGUTIL_H

#include <chalk/chalk.h>
#include <date/date.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <format>
#include <functional>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#if (__cplusplus >= 202002L)
template <ssize_t N>
consteval auto _basename(const char (&path)[N]) {
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

namespace logger {
namespace details {

template <typename... Args>
static inline auto fmt_args(auto &&fmt_str, Args &&...args) {
  return (std::vformat(std::forward<decltype(fmt_str)>(fmt_str),
                       std::make_format_args(std::forward<Args>(args)...)));
}

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

template <ssize_t N>
consteval std::string basename(const char (&path)[N]) {
  ssize_t idx;
  for (idx = N - 1; idx >= 0; --idx)
    if (path[idx] == '/') break;
  return (path + idx + 1);
}

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

}  // namespace logger
template <typename Clock = std::chrono::system_clock>
static inline auto isoDate(std::chrono::time_point<Clock> tp = Clock::now())
    -> std::string {
  return date::format("%FT%TZ",
                      std::chrono::floor<std::chrono::milliseconds>(tp));
}

struct Logger {
  explicit Logger(std::ostream &ost = std::cout, std::ostream &est = std::cerr)
      : outStream_{&ost}, errStream_{&est} {}

  template <typename... Args>
  inline auto Info(Args &&...args) {
    if (verbosity_ >= Verbosity::kInfo)
      output(std::chrono::system_clock::now(), outStream_, kInfoChalk,
             std::forward<Args>(args)...);
  }

  template <const char *FmtStr, typename... Args>
  inline auto InfoConstFmt(Args &&...args) {
    if (verbosity_ >= Verbosity::kInfo)
      output<FmtStr>(std::chrono::system_clock::now(), outStream_, kInfoChalk,
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

#ifndef NDEBUG
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
  auto output(std::chrono::time_point<Clock> &&tpt, std::ostream *ost,
              FmtType &&fmt, auto &&fmt_str, Args &&...args) {
    const std::chrono::duration<float> delta_t =
        tpt - ((ost == outStream_ && !this->unified_output_) ? lastTimes_[0]
                                                             : lastTimes_[1]);

    lastTimes_[(ost == outStream_ && !this->unified_output_) ? 0 : 1] = tpt;

    *ost << (kTsChalk(isoDate(tpt)) + "  " +
             fmt(logger::details::fmt_args(
                 std::forward<decltype(fmt_str)>(fmt_str),
                 std::forward<Args>(args)...)) +
             " +" + kDtChalk(std::to_string(delta_t.count()) + "s") + "\n");
  }

 private:
  std::ostream *outStream_, *errStream_;
  std::chrono::time_point<std::chrono::system_clock> lastTimes_[2]{
      std::chrono::system_clock::now(), std::chrono::system_clock::now()};

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
      chalk::compose(chalk::fmt::Italic, chalk::fg::Blue);

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
  static auto get() noexcept -> Logger & {
    try {
      static auto singleton = []() {
        auto my_pid = getpid();
        constexpr auto kSzLogger = sizeof(std::shared_ptr<Logger>);

        if (const int shm_fd =
                shm_open(std::format("/{}-logger", my_pid).c_str(), O_RDWR,
                         S_IRUSR | S_IWUSR);
            shm_fd != -1) {
          auto *mmapped_shm = static_cast<std::shared_ptr<Logger> *>(
              mmap(nullptr, kSzLogger, PROT_READ | PROT_WRITE, MAP_SHARED,
                   shm_fd, 0));

          if (mmapped_shm == MAP_FAILED)
            throw std::runtime_error{std::format(
                "{}:{}:{}: on-restore mmap of shm area failed: {}", __FILE__,
                __LINE__, __PRETTY_FUNCTION__, strerror(errno))};

          close(shm_fd);
          auto prev_logger = *mmapped_shm;

          munmap(mmapped_shm, kSzLogger);
          return prev_logger;
        }

        if (const int shm_fd =
                shm_open(std::format("/{}-logger", my_pid).c_str(),
                         O_CREAT | O_EXCL | O_RDWR, S_IRUSR | S_IWUSR);
            shm_fd != -1) {
          if (ftruncate(shm_fd, kSzLogger) == -1) {
            throw std::runtime_error{
                std::format("{}:{}:{}: ftruncate failed: {}", __FILE__,
                            __LINE__, __PRETTY_FUNCTION__, strerror(errno))};
          }
          auto new_logger = std::make_shared<Logger>();

          auto *mmapped_shm = static_cast<std::shared_ptr<Logger> *>(
              mmap(nullptr, kSzLogger, PROT_READ | PROT_WRITE, MAP_SHARED,
                   shm_fd, 0));

          if (mmapped_shm == MAP_FAILED)
            throw std::runtime_error{std::format(
                "{}:{}:{}: on-create mmap of shm area failed: {}", __FILE__,
                __LINE__, __PRETTY_FUNCTION__, strerror(errno))};

          *mmapped_shm = new_logger;

          close(shm_fd);
          munmap(mmapped_shm, kSzLogger);

          return new_logger;
        }

        throw std::runtime_error{
            std::format("{}:{}:{}: unable to restore or create a new 'shm' "
                        "handle for Logger",
                        __FILE__, __LINE__, __PRETTY_FUNCTION__)};
      }();
      return *singleton;
    } catch (std::exception const &err) {
      std::cerr << std::format(
          "{}:{}:{}: caught exception while constructing Logger singleton: "
          "{}\n",
          __FILE__, __LINE__, __func__, err.what());
      std::terminate();
    }
  }
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

#define INFO(_fmt, ...)                                                        \
  do {                                                                         \
    logger.Info(                                                               \
        logger::zstrcats(logger::zatoa("["),                                   \
                         logger::zatoa(logger::PastLastSlash<__FILE__>::kArr), \
                         logger::zatoa(":" QUOTE(__LINE__) ":"),               \
                         logger::zatoa(__func__), logger::zatoa("]: [II] "),   \
                         logger::zatoa(_fmt))                                  \
                                                                               \
            .data() __VA_OPT__(, ) __VA_ARGS__);                               \
  } while (false)

#define ERROR(_fmt, ...)                                                       \
  do {                                                                         \
    logger.Error(                                                              \
        logger::zstrcats(logger::zatoa("[" __FILE__ ":" QUOTE(__LINE__) ":"),  \
                         logger::zatoa(logger::PastLastSlash<__FILE__>::kArr), \
                         logger::zatoa("]: [EE] "), logger::zatoa(_fmt))       \
                                                                               \
            .data() __VA_OPT__(, ) __VA_ARGS__);                               \
  } while (false)

#define WARN(_fmt, ...)                                                        \
  do {                                                                         \
    logger.Warn(                                                               \
        logger::zstrcats(logger::zatoa("[" __func__ ":" QUOTE(__LINE__) ":"),  \
                         logger::zatoa(logger::PastLastSlash<__FILE__>::kArr), \
                         logger::zatoa("]: [WW] "), logger::zatoa(_fmt))       \
                                                                               \
            .data() __VA_OPT__(, ) __VA_ARGS__);                               \
  } while (false)

#define DEBUG(_fmt, ...)                                                       \
  do {                                                                         \
    logger.Debug(                                                              \
        logger::zstrcats(logger::zatoa("[" __FILE__ ":" QUOTE(__LINE__) ":"),  \
                         logger::zatoa(logger::PastLastSlash<__FILE__>::kArr), \
                         logger::zatoa("]: [DD] "), logger::zatoa(_fmt))       \
                                                                               \
            .data() __VA_OPT__(, ) __VA_ARGS__);                               \
  } while (false)

#ifdef NDEBUG
#define TRACE(...) \
  { ; }
#else
#define TRACE(_fmt, ...)                                                       \
  do {                                                                         \
    logger.Trace(                                                              \
        logger::zstrcats(logger::zatoa("[" __FILE__ ":" QUOTE(__LINE__) ":"),  \
                         logger::zatoa(logger::PastLastSlash<__FILE__>::kArr), \
                         logger::zatoa("]: [TT] "), logger::zatoa(_fmt))       \
                                                                               \
            .data() __VA_OPT__(, ) __VA_ARGS__);                               \
  } while (false)

#endif

#endif  // TH2_LOGUTIL_H
