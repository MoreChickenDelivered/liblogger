// (C) 2019 TT.io
// Author: Elias Benali <ebenali@tradeterminal.io>
// Created by deb on 11/23/19.
//

#include <dlfcn.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>

#include "logutil.h"

int main(int argc, char *argv[]) {
  std::srand(std::time(nullptr));
  auto &logger = Logger::get();  // default logger

  auto hd_logfile = std::ofstream{std::to_string(std::time(nullptr)) + ".txt"};
  auto hd_logger = Logger{hd_logfile, hd_logfile};
  hd_logger.setVerbosity(Logger::Verbosity::kTrace);

  struct randInts {
    randInts() = default;
    explicit randInts(ssize_t n) : n_{n} {}
    bool operator==(randInts const &rhs) const { return rhs.n_ == n_; }
    bool operator!=(randInts const &rhs) const { return !(*this == rhs); }
    randInts &operator++() {
      --n_;
      return *this;
    }
    int operator*() const { return std::rand() * ((std::rand() % 2) ? -1 : 1); }

   private:
    ssize_t n_{};
  };

  std::for_each(
      randInts(5), randInts(), [&](int i) {  // by-ref capture for 'logger'
        // raw lines, console.debug alike, does not support fmt::format syntax
        hd_logger.Trace("random integer:", i);
        // using fmt::format
        hd_logger.Trace(fmt::format("hig-res timestamp: {}ns",
                                    std::chrono::high_resolution_clock::now()
                                        .time_since_epoch()
                                        .count()));

        // managed logger, pretty prints
        DEBUG("random int squared: {}", i * i);
        // will not be output to stderr, since default logger's verbosity is
        // DEBUG
        TRACE("random int: {}", i);
      });

  logger.setVerbosity(Logger::Verbosity::kTrace);
  TRACE("current dir: {}", std::filesystem::current_path().native());
  auto *dlhandle = dlopen("./libexample_dylib.so", RTLD_NOW | RTLD_GLOBAL);
  if (!dlhandle)
    throw std::runtime_error(fmt::format("dlopen failed: {}", dlerror()));
  void (*libmain_fn)() =
      reinterpret_cast<void (*)()>(dlsym(dlhandle, "libmain"));
  if (!libmain_fn)
    throw std::runtime_error(fmt::format("dlsym failed: {}", dlerror()));
  libmain_fn();
  dlclose(dlhandle);
}
