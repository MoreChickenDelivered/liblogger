// (C) 2019 TT.io
// Author: Elias Benali <ebenali@tradeterminal.io>
// Created by deb on 11/23/19.
//

#include <dlfcn.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/wait.h>

#include <algorithm>
#include <chrono>
#include <cpptrace/cpptrace.hpp>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <format>
#include <fstream>
#include <random>
#include <ranges>

#include "logutil.h"

int main(int /*argc*/, char * /*argv*/[]) {
  cpptrace::register_terminate_handler();

  static auto &logger = Logger::get();  // default logger

  static auto flogger = [] {
    auto const ofs_path = (std::filesystem::temp_directory_path() /
                           std::to_string(std::time(nullptr)))
                              .replace_extension(".txt");
    auto ofs_fd = open(ofs_path.c_str(), O_CREAT | O_WRONLY | O_TRUNC,
                       S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
    RELEASE_ASSERT(ofs_fd != -1, "create hd-sinked logger failed: {}",
                   strerror(errno));
    return Logger{ofs_fd, ofs_fd};
  }();

  flogger.setVerbosity(Logger::Verbosity::kTrace);
  flogger.setUnifiedOutput(true);

  (void)std::ranges::for_each(
      std::views::iota(0) | std::views::take(2) |
          std::views::transform([](auto) {
            static std::random_device rdev{};
            static std::default_random_engine rng{rdev()};
            static std::uniform_int_distribution<> dist{-1000, 1000};
            return dist(rng);
          }),
      [&](int cur_int) {
        // raw lines, console.debug alike, does not support
        // std::format syntax
        flogger.Info("random integer: {}", cur_int);

        // using std::format
        auto pdd = std::chrono::high_resolution_clock::now();
        flogger.Trace(std::format("hig-res timestamp: {}ns",
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

  // tests RELEASE_ASSERT
  Logger::flush();
  if (auto is_ppid = fork(); !is_ppid) {
    RELEASE_ASSERT(
        !"unsuccessul-run",
        "\n\n\tTHIS IS NOT A REAL FAILURE.\n\tIT TRIGGERS A CONTROLLED "
        "ABORT FROM A SPAWNED CHILD PROCESS.\n");
  } else {
    waitpid(is_ppid, nullptr, 0);
    INFO("OK");
  }
}