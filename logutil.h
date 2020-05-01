//
// (C) 2019 TT.io
// Author: Elias Benali <ebenali@tradeterminal.io>
// Created by deb on 11/23/19.
//

#ifndef TH2_LOGUTIL_H
#define TH2_LOGUTIL_H

#include <iostream>
#include <sstream>
#include <chrono>
#include <mutex>
#include <string>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include <chalk/chalk.h>
#include <date/date.h>

template<typename Clock = std::chrono::system_clock>
static inline std::string isoDate(std::chrono::time_point<Clock> tp = Clock::now()) {
    return date::format("%FT%TZ", std::chrono::floor<std::chrono::milliseconds>(tp));
}

struct Logger {
    Logger(std::ostream &ost=std::cout, std::ostream &est=std::cerr): outStream{&ost}, errStream{&est} {}
    template<typename ..._Args>
    std::ostream &Info(const _Args &... args) {
        return (verbosity_ >= Verbosity::INFO)
            ? output(std::chrono::system_clock::now(), outStream, infoChalk, "[II]", args...)
            : *outStream;
    }

    template<typename ..._Args>
    std::ostream &Warn(const _Args &... args) {
        return (verbosity_ >= Verbosity::WARN)
               ? output(std::chrono::system_clock::now(), outStream, warnChalk, "[WW]", args...)
               : *outStream;
    }

    template<typename ..._Args>
    std::ostream &Error(const _Args &... args) {
        return (verbosity_ >= Verbosity::ERR)
               ? output(std::chrono::system_clock::now(), errStream, errorChalk, "[EE]", args...)
               : *errStream;
    }
    template<typename ..._Args>
    std::ostream &Debug(const _Args &... args) {
        return (verbosity_ >= Verbosity::DEBUG)
               ? output(std::chrono::system_clock::now(), errStream, debugChalk, "[DD]", args...)
               : *errStream;
    }
    template<typename ..._Args>
    std::ostream &Trace(const _Args &... args) {
        return (verbosity_ >= Verbosity::TRACE)
               ? output(std::chrono::system_clock::now(), errStream, traceChalk, "[TT]", args...)
               : *errStream;
    }

    void setUnifiedOutput(bool flag) { this->unified_output_ = flag; }

    enum class Verbosity: unsigned {
        QUIET = 0,
        ERR,
        WARN,
        INFO,
        DEBUG,
        TRACE,
    };

    void setVerbosity(Verbosity verbosity) { verbosity_ = verbosity; }

protected:
    template<typename Clock, typename _fmtType, typename ..._Args>
    std::ostream &output(std::chrono::time_point<Clock> &&tm, std::ostream *ost, const _fmtType &fmt = chalk::fg::White, const _Args&... args) {
        std::lock_guard<std::mutex> lk{(ost == outStream && !this->unified_output_) ? outMutex : errMutex };
        const std::chrono::duration<float> deltaT = tm - ((ost == outStream && !this->unified_output_) ? lastTimes[0] : lastTimes[1]);
        lastTimes[(ost == outStream && !this->unified_output_) ? 0 : 1] = tm;
        *ost << tsChalk(isoDate(tm)) << "  " << fmt(logN(args...)) << " +" << dtChalk(std::to_string(deltaT.count()) + "s") << std::endl;
        return *ost;
    }
    template<typename _Arg0, typename ...Args>
    std::string logN(_Arg0 const &arg0, const Args&... args) {
        std::ostringstream oss{};
        oss << " " << arg0;
        return oss.str() + logN(args...);
    }
    std::string logN() {
        return "";
    }
private:
    std::ostream *outStream, *errStream;
    std::chrono::time_point<std::chrono::system_clock> lastTimes[2] { std::chrono::system_clock::now(), std::chrono::system_clock::now() };
    inline static std::mutex outMutex, errMutex;

    bool unified_output_ = false;
    Verbosity verbosity_ = Verbosity::DEBUG;

    static constexpr auto tsChalk = chalk::compose(chalk::fg::Green, chalk::fmt::Bold);
    static constexpr auto dtChalk = chalk::compose(chalk::fg::BrightCyan, chalk::fmt::Bold);
    static constexpr auto infoChalk = chalk::fg::White;
    static constexpr auto errorChalk = chalk::fg::Red;
    static constexpr auto warnChalk = chalk::fg::Yellow;
    static constexpr auto debugChalk = chalk::fg::Magenta;
    static constexpr auto traceChalk = chalk::compose(chalk::fmt::Italic, chalk::fg::Blue);

    inline static std::unique_ptr<Logger> singleton = std::make_unique<Logger>();
    public:
        /**
         * singleton style
         */
         static Logger &get() {
             return *singleton;
         }
};

//extern Logger logger;

using namespace std::string_literals;
inline const char *_basename(const char *path) {
    const char *p = path + strlen(path);
    while (p > path && *p != '/')
        --p;
    return p == path ? path : p + 1;
}
#  define INFO(_fmt,...) do { logger.Info(fmt::format("[{}:{}:{}]: "s + _fmt, __func__, _basename(__FILE__), __LINE__ __VA_OPT__(,) __VA_ARGS__ )); } while (false);
#  define ERROR(_fmt,...) do { logger.Error(fmt::format("[{}:{}:{}]: "s + _fmt, __func__, _basename(__FILE__), __LINE__ __VA_OPT__(,) __VA_ARGS__ )); } while (false);
#  define WARN(_fmt,...) do { logger.Warn(fmt::format("[{}:{}:{}]: "s + _fmt, __func__, _basename(__FILE__), __LINE__ __VA_OPT__(,) __VA_ARGS__ )); } while (false);
#  define DEBUG(_fmt,...) do { logger.Debug(fmt::format("[{}:{}:{}]: "s + _fmt, __func__, _basename(__FILE__), __LINE__ __VA_OPT__(,) __VA_ARGS__)); } while (false);

#  ifdef NDEBUG
#   define TRACE(...) {;}
#  else
#   define TRACE(_fmt,...) do { logger.Trace(fmt::format("[{}:{}:{}]: "s + _fmt, __func__, _basename(__FILE__), __LINE__ __VA_OPT__(,) __VA_ARGS__)); } while (false);
#  endif

#endif //TH2_LOGUTIL_H
