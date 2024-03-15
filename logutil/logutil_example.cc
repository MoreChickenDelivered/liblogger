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
#include <format>
#include <fstream>
#include <ranges>

#include "logutil.h"

int main(int argc, char *argv[]) {
  std::srand(std::time(nullptr));
  static auto &logger = Logger::get();  // default logger

  std::cout << __FILE__ "\n";

  static auto hd_logfile =
      std::ofstream{std::to_string(std::time(nullptr)) + ".txt"};
  static auto hd_logger = Logger{hd_logfile, hd_logfile};
  hd_logger.setVerbosity(Logger::Verbosity::kTrace);

  struct RandInts {
    RandInts() = default;
    explicit RandInts(ssize_t n) : n_{n} {}
    bool operator==(RandInts const &rhs) const { return rhs.n_ == n_; }
    bool operator!=(RandInts const &rhs) const { return !(*this == rhs); }
    RandInts &operator++() {
      --n_;
      return *this;
    }
    int operator*() const { return std::rand() * ((std::rand() % 2) ? -1 : 1); }

   private:
    ssize_t n_{};
  };

  (void)std::ranges::for_each(
      std::views::iota(0, 1) | std::views::transform([](auto) {
        return std::rand() * ((std::rand() % 2) ? -1 : 1);
      }),
      [](int cur_int) {
        // raw lines, console.debug alike, does not support
        // std::format syntax
        hd_logger.Trace("random integer: {}", cur_int);

        // using std::format
        auto pdd = std::chrono::high_resolution_clock::now();
        dprintf(1, "%zu\n", pdd.time_since_epoch().count());
        hd_logger.Trace(std::format("hig-res timestamp: {}ns",
                                    std::chrono::high_resolution_clock::now()
                                        .time_since_epoch()
                                        .count()));

        DEBUG("random int: {}", cur_int);
        INFO("random int squared: {}", cur_int * cur_int);
      });

  // non-constexpr formatting
  std::string non_constexpr_format;

  if (std::rand() & 0b1)
    non_constexpr_format += "camelCase42={}";
  else
    non_constexpr_format += "snake_case_42={}";

  INFO(non_constexpr_format, 42);

  logger.setVerbosity(Logger::Verbosity::kTrace);

  TRACE("current dir: {}", std::filesystem::current_path().native());

  auto *dlhandle = dlopen("./libexample_dylib.so", RTLD_NOW | RTLD_GLOBAL);
  if (!dlhandle)
    throw std::runtime_error(std::format("dlopen failed: {}", dlerror()));

  auto *libmain_fn = reinterpret_cast<void (*)()>(dlsym(dlhandle, "libmain"));
  if (!libmain_fn)
    throw std::runtime_error(std::format("dlsym failed: {}", dlerror()));

  libmain_fn();

  dlclose(dlhandle);
}
