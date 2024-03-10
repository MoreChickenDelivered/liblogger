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
#include <format>
#include <iostream>
#include <sstream>
#include <string>

template <typename Clock = std::chrono::system_clock>
static inline auto isoDate(std::chrono::time_point<Clock> tp = Clock::now())
    -> std::string {
  return date::format("%FT%TZ",
                      std::chrono::floor<std::chrono::milliseconds>(tp));
}

struct Logger {
  explicit Logger(std::ostream &ost = std::cout, std::ostream &est = std::cerr)
      : outStream_{&ost}, errStream_{&est} {}
  template <typename... _Args>
  auto Info(const _Args &...args) -> std::ostream & {
    return (verbosity_ >= Verbosity::kInfo)
               ? output(std::chrono::system_clock::now(), outStream_,
                        kInfoChalk, "[II]", args...)
               : *outStream_;
  }

  template <typename... _Args>
  auto Warn(const _Args &...args) -> std::ostream & {
    return (verbosity_ >= Verbosity::kWarn)
               ? output(std::chrono::system_clock::now(), outStream_,
                        kWarnChalk, "[WW]", args...)
               : *outStream_;
  }

  template <typename... _Args>
  auto Error(const _Args &...args) -> std::ostream & {
    return (verbosity_ >= Verbosity::kErr)
               ? output(std::chrono::system_clock::now(), errStream_,
                        kErrorChalk, "[EE]", args...)
               : *errStream_;
  }
  template <typename... _Args>
  auto Debug(const _Args &...args) -> std::ostream & {
    return (verbosity_ >= Verbosity::kDebug)
               ? output(std::chrono::system_clock::now(), errStream_,
                        kDebugChalk, "[DD]", args...)
               : *errStream_;
  }
  template <typename... _Args>
  auto Trace(const _Args &...args) -> std::ostream & {
    return (verbosity_ >= Verbosity::kTrace)
               ? output(std::chrono::system_clock::now(), errStream_,
                        kTraceChalk, "[TT]", args...)
               : *errStream_;
  }

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
  template <typename Clock, typename FmtType, typename... _Args>
  auto output(std::chrono::time_point<Clock> &&tm, std::ostream *ost,
              const FmtType &fmt = chalk::fg::White, const _Args &...args)
      -> std::ostream & {
    const std::chrono::duration<float> delta_t =
        tm - ((ost == outStream_ && !this->unified_output_) ? lastTimes_[0]
                                                            : lastTimes_[1]);
    lastTimes_[(ost == outStream_ && !this->unified_output_) ? 0 : 1] = tm;
    *ost << (kTsChalk(isoDate(tm)) + "  " + fmt(logN(args...)) + " +" +
             kDtChalk(std::to_string(delta_t.count()) + "s") + "\n");
    return *ost;
  }
  template <typename _Arg0, typename... Args>
  auto logN(_Arg0 const &arg0, const Args &...args) -> std::string {
    std::ostringstream oss{};
    oss << " " << arg0;
    return oss.str() + logN(args...);
  }
  static auto logN() -> std::string { return ""; }

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
   * singleton style
   */
  static auto get() noexcept -> Logger & {
    try {
      static std::shared_ptr<Logger> singleton = []() {
        auto mypid = getpid();
        constexpr auto kSzLogger = sizeof(std::shared_ptr<Logger>);
        if (int shm_fd = shm_open(std::format("/{}-logger", mypid).c_str(),
                                  O_CREAT | O_EXCL | O_RDWR, S_IRUSR | S_IWUSR);
            shm_fd != -1) {
          ftruncate(shm_fd, kSzLogger);
          auto new_logger = std::make_shared<Logger>();
          auto *plogger = static_cast<std::shared_ptr<Logger> *>(
              mmap(nullptr, kSzLogger, PROT_READ | PROT_WRITE, MAP_SHARED,
                   shm_fd, 0));
          if (plogger == MAP_FAILED)
            throw std::runtime_error{std::format("shmlog mmap fail")};
          *plogger = new_logger;
          close(shm_fd);
          // std::clog << std::format(
          //     "save logger shmem@/{}-logger; ptr={:p}; sptr.p={:p}\n", mypid,
          //     (void *)(plogger), (void *)plogger->get());
          munmap(plogger, kSzLogger);
          return new_logger;
        } else if (int shm_fd =
                       shm_open(std::format("/{}-logger", mypid).c_str(),
                                O_RDWR, S_IRUSR | S_IWUSR);
                   shm_fd != -1) {
          auto *plogger = static_cast<std::shared_ptr<Logger> *>(
              mmap(nullptr, kSzLogger, PROT_READ | PROT_WRITE, MAP_SHARED,
                   shm_fd, 0));
          if (plogger == MAP_FAILED)
            throw std::runtime_error{std::format("shmlog mmap fail")};
          close(shm_fd);
          auto prev_logger = *plogger;
          // std::clog << std::format(
          //     "restore logger shmem@/{}-logger; ptr={:p}; sptr.p={:p}\n",
          //     mypid, (void *)(plogger), (void *)plogger->get());
          munmap(plogger, kSzLogger);
          return prev_logger;
        } else {
          throw std::runtime_error{std::format("shm logger open: {}{}{}",
                                               __FILE__, __LINE__, __func__)};
        }
        // return std::make_unique<Logger>();
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

// extern Logger logger;

#if (__cplusplus >= 202002L)
template <ssize_t N>
consteval auto _basename(const char (&path)[N]) -> const char * {
  ssize_t i;
  for (i = N - 1; i >= 0; --i)
    if (path[i] == '/') break;
  return &(path[i + 1]);
}
#else
inline const char *_basename(const char *path) {
  const char *p = path + strlen(path);
  while (p > path && *p != '/') --p;
  return p == path ? path : p + 1;
}
#endif

using namespace std::string_literals;
#define INFO(_fmt, ...)                                            \
  do {                                                             \
    logger.Info(std::format("[{}:{}:{}]: " _fmt, __func__,         \
                            _basename(__FILE__),                   \
                            __LINE__ __VA_OPT__(, ) __VA_ARGS__)); \
  } while (false);
#define ERROR(_fmt, ...)                                            \
  do {                                                              \
    logger.Error(std::format("[{}:{}:{}]: " _fmt, __func__,         \
                             _basename(__FILE__),                   \
                             __LINE__ __VA_OPT__(, ) __VA_ARGS__)); \
  } while (false);
#define WARN(_fmt, ...)                                            \
  do {                                                             \
    logger.Warn(std::format("[{}:{}:{}]: " _fmt, __func__,         \
                            _basename(__FILE__),                   \
                            __LINE__ __VA_OPT__(, ) __VA_ARGS__)); \
  } while (false);
#define DEBUG(_fmt, ...)                                            \
  do {                                                              \
    logger.Debug(std::format("[{}:{}:{}]: " _fmt, __func__,         \
                             _basename(__FILE__),                   \
                             __LINE__ __VA_OPT__(, ) __VA_ARGS__)); \
  } while (false);

#ifdef NDEBUG
#define TRACE(...) \
  { ; }
#else
#define TRACE(_fmt, ...)                                            \
  do {                                                              \
    logger.Trace(std::format("[{}:{}:{}]: " _fmt, __func__,         \
                             _basename(__FILE__),                   \
                             __LINE__ __VA_OPT__(, ) __VA_ARGS__)); \
  } while (false);
#endif

#endif  // TH2_LOGUTIL_H
