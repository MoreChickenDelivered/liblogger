#pragma once

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <boost/iostreams/filter/bzip2.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filter/lzma.hpp>
#include <boost/iostreams/filter/zstd.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <format>
#include <fstream>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <utility>

#include "../logutil/logutil.h"

static FILE *const kCons = []() {
  const int cons_fd = open("/dev/tty", O_WRONLY);
  return cons_fd > -1 ? fdopen(cons_fd, "w") : stdout;
}();

/**
 * @class INTST
 * @brief The INTST class represents an inspector that logs and saves internal
 * state information.
 *
 * The INTST class is responsible for logging and saving internal state
 * information. It provides a thread-safe mechanism for capturing and storing
 * changes in the state of an object. The class uses a shared pointer to manage
 * the internal snapshot file descriptor and provides methods for initializing
 * the inspector, setting the file descriptor, saving the state, and retrieving
 * the singleton instance.
 *
 * The INTST class uses a logger thread to continuously monitor the state
 * changes and save them to the internal snapshot file. It supports compression
 * options for the snapshot file and allows customization through a
 * configuration object.
 */
class INTST {
  static auto constexpr kFlushEveryNCycles = 1000;

 public:
  uint64_t era = 0;
  nlohmann::json kwargs;

  // std::optional<int> internal_snapshot_fd_ = std::nullopt;
  std::shared_ptr<boost::iostreams::filtering_ostream> internal_snapshot_fd_{
      nullptr};

  std::shared_ptr<std::atomic_flag> kwargs_mutex_ =
      std::make_shared<std::atomic_flag>();
  std::shared_ptr<std::atomic_bool> close_sig_ =
      std::make_shared<std::atomic_bool>(false);
  std::shared_ptr<std::deque<nlohmann::json>> kwargs_queue_ =
      std::make_shared<std::deque<nlohmann::json>>();
  std::shared_ptr<std::thread> logger_thread_ = nullptr;

  ~INTST() {
    // if (this->internal_snapshot_fd_.has_value())
    //   ::close(this->internal_snapshot_fd_.value());
    if (this->close_sig_ != nullptr) *(this->close_sig_) = true;
    if (this->logger_thread_ && this->logger_thread_.use_count() == 1) {
      // fmt::println(cons_, "Waiting form INTST logger thread to finish...");
      this->logger_thread_->join();
    }
  }

  INTST() = default;

  void init() {
    {
      if (!this->logger_thread_)
        this->logger_thread_ = std::make_shared<std::thread>(
            [=, this, fdesc = internal_snapshot_fd_, cons = kCons] {
              static thread_local nlohmann::json prev_kwargs{};
              static thread_local nlohmann::json cur_kwargs{};

              // fmt::println(cons_, "INTST thread started");

            out_loop:
              while (true) {
                if (*close_sig_) break;

                while (kwargs_mutex_->test_and_set(std::memory_order_acquire)) {
                  while (kwargs_mutex_->test(std::memory_order_relaxed)) {
                    if (*close_sig_) {
                      kwargs_mutex_->clear();
                      goto out_loop;
                    }
                  }
                }

                if (kwargs_queue_->empty()) {
                  kwargs_mutex_->clear();
                  constexpr auto kKwargsThreadSpinnerSleepDuration =
                      std::chrono::milliseconds(10);

                  std::this_thread::sleep_for(
                      kKwargsThreadSpinnerSleepDuration);
                  continue;
                }

                // fmt::println(cons_, "INTST thread: got KWARGS ({})",
                //              kwargs_queue_->size());

                while (!kwargs_queue_->empty()) {
                  if (*close_sig_) {
                    kwargs_mutex_->clear();
                    goto out_loop;
                  }

                  nlohmann::json diff_kwargs{};

                  {
                    cur_kwargs = std::move(kwargs_queue_->front());
                    kwargs_queue_->pop_front();
                    kwargs_mutex_->clear();
                  }

                  std::optional<uint64_t> era = std::nullopt;
                  std::optional<std::string> cur_ts = std::nullopt;

                  if (cur_kwargs.contains("era")) {
                    era = cur_kwargs["era"].get<uint64_t>();
                    cur_kwargs.erase("era");
                  }

                  if (cur_kwargs.contains("timestamp")) {
                    const auto dbl_ts = cur_kwargs["timestamp"].get<double>();
                    auto const chr_ts = std::chrono::duration_cast<
                        std::chrono::system_clock::duration>(
                        std::chrono::duration<double, std::ratio<1>>(dbl_ts));

                    auto const chr_tp = std::chrono::time_point<
                        std::chrono::system_clock,
                        std::chrono::system_clock::duration>(chr_ts);

                    cur_ts = isoDate(chr_tp);

                    cur_kwargs.erase("timestamp");
                  }

                  diff_kwargs =
                      nlohmann::json::diff(prev_kwargs, cur_kwargs, "");

                  prev_kwargs = std::move(cur_kwargs);

                  if (diff_kwargs.empty()) {
                    continue;
                  }

                  auto aug_kwargs = nlohmann::json::array({});
                  if (era.has_value()) aug_kwargs.push_back(era.value());
                  if (cur_ts.has_value())
                    aug_kwargs.emplace_back(std::move(cur_ts.value()));
                  aug_kwargs.emplace_back(std::move(diff_kwargs));

                  auto fmted_kwargs = aug_kwargs.dump() + "\n\n";
                  *fdesc << fmted_kwargs;

                  if (era.has_value() && era.value() % kFlushEveryNCycles == 0)
                    fdesc->flush();
                }
              }

              // fmt::println(cons_,
              //              "{}: main loop concluded. syncing output if
              //              any...",
              //              __PRETTY_FUNCTION__);
              fdesc->flush();
            });
      this->logger_thread_->detach();
    }
  }

  void set_fd(std::shared_ptr<boost::iostreams::filtering_ostream> bobf) {
    this->internal_snapshot_fd_.swap(bobf);
  }

  void save_state_and_reset(double timestamp) {
    this->kwargs["era"] = era++;

    static thread_local auto fmted_state_line = this->kwargs.dump(-1) + "\n";

    while (kwargs_mutex_->test_and_set(std::memory_order_acquire)) {
      while (kwargs_mutex_->test(std::memory_order_relaxed)) {
        // if (*this->close_sig_) break;
      }
    }

    if (*this->close_sig_) {
      // fmt::println(cons_, "EX:IT SIGNAL; KSWARG: {}", this->kwargs.dump());
      kwargs_mutex_->clear();
      return;
    }

    this->kwargs["timestamp"] = timestamp;
    this->kwargs_queue_->push_back(std::move(this->kwargs));

    kwargs_mutex_->clear();
  }

  /**
   * @brief Singleton factory function to get the INTST instance.
   *
   * This function returns a singleton instance of INTST. It takes a config
   * pointer as an argument, which can be used to customize the behavior of the
   * INTST instance.
   *
   * @param config A pointer to a JSON object containing configuration options.
   *     The following keys are accepted:
   *     - "save_internal_state": Enable saving internal state.
   *     - "save_internal_state_compress": Compression format for the
   *       saved internal state (optional).
   *     - "save_internal_state_basedir": Directory to save the internal state
   * dump (optional).
   *
   * @return The singleton instance of INTST.
   */
  static std::shared_ptr<INTST> get(nlohmann::json const *config) noexcept {
    try {
      const static std::shared_ptr<INTST> kSingleton = [&]() {
        const auto mypid = getpid();
        constexpr auto kSzIntSt = sizeof(std::shared_ptr<INTST>);

        // Create a shared memory segment for the singleton instance
        if (const int shm_fd =
                shm_open(std::format("/{}-intst", mypid).c_str(),
                         O_CREAT | O_EXCL | O_RDWR, S_IRUSR | S_IWUSR);
            shm_fd != -1) {
          (void)ftruncate(shm_fd, kSzIntSt);

          auto new_intst = std::make_shared<INTST>();

          // Map the shared memory segment to the singleton instance
          auto *p_intst = static_cast<std::shared_ptr<INTST> *>(
              mmap(nullptr, kSzIntSt, PROT_READ | PROT_WRITE, MAP_SHARED,
                   shm_fd, 0));

          if (p_intst == MAP_FAILED)
            throw std::runtime_error{std::format("{}:{}: shmlog mmap fail: {}",
                                                 __FILE__, __LINE__,
                                                 strerror(errno))};

          *p_intst = new_intst;
          close(shm_fd);
          munmap(p_intst, kSzIntSt);

          // Check if config is null or if save_internal_state option is present
          if (config == nullptr || !config->contains("save_internal_state") ||
              !(*config)["save_internal_state"].get<bool>()) {
            fprintf(kCons, "%s\n",
                    std::format("{}:{}:{}: internal-state configuration not "
                                "present or explicitly disabled",
                                __FILE__, __LINE__, __PRETTY_FUNCTION__)
                        .c_str());
            return new_intst;
          }

          // Get the compression option from the config
          auto compress = config->contains("save_internal_state_compress")
                              ? (*config)["save_internal_state_compress"]
                                    .get<std::string_view>()
                              : "";

          // Get the dump directory from the config
          auto const dump_dir = config->contains("save_internal_state_basedir")
                                    ? (*config)["save_internal_state_basedir"]
                                          .get<std::string_view>()
                                    : "/tmp";

          // Generate the filename for the internal state dump
          auto const fname = std::format("{}/{}-internal.json{}", dump_dir,
                                         isoDate(), compress);

          // Create an output file stream for the dump file
          static std::unique_ptr<std::ofstream> out_stdfile{};
          out_stdfile = std::make_unique<std::ofstream>(
              fname, std::ios::out | std::ios::trunc | std::ios::binary);

          if (!out_stdfile || !*out_stdfile)
            throw std::runtime_error{
                std::format("{}:{}: can't create file {}: {}", __FILE__,
                            __LINE__, fname, ::strerror(errno))};

          // Create a filtering output stream with compression options
          auto out_sbuf =
              std::make_shared<boost::iostreams::filtering_ostream>();

          if (compress == ".xz")
            out_sbuf->push(boost::iostreams::lzma_compressor());
          if (compress == ".gz")
            out_sbuf->push(boost::iostreams::gzip_compressor());
          if (compress == ".bz2")
            out_sbuf->push(boost::iostreams::bzip2_compressor());
          if (compress == ".zstd")
            out_sbuf->push(boost::iostreams::zstd_compressor());

          out_sbuf->push(*out_stdfile);

          // Set the output stream for the INTST instance
          new_intst->set_fd(std::move(out_sbuf));

          new_intst->init();
          return new_intst;
        }

        // If the shared memory segment already exists, restore the previous
        // instance
        if (const int shm_fd = shm_open(std::format("/{}-intst", mypid).c_str(),
                                        O_RDWR, S_IRUSR | S_IWUSR);
            shm_fd != -1) {
          auto *p_intst = static_cast<std::shared_ptr<INTST> *>(
              mmap(nullptr, kSzIntSt, PROT_READ | PROT_WRITE, MAP_SHARED,
                   shm_fd, 0));

          if (p_intst == MAP_FAILED)
            throw std::runtime_error{std::format("{}:{}: shm-mmap fail: {}",
                                                 __FILE__, __LINE__,
                                                 ::strerror(errno))};

          close(shm_fd);
          auto prev_intst = *p_intst;
          munmap(p_intst, kSzIntSt);

          return prev_intst;
        }

        throw std::runtime_error{std::format(
            "{}:{}:{}: shm_open(path='/{}-intst') failed: {}", __FILE__,
            __LINE__, __PRETTY_FUNCTION__, mypid, ::strerror(errno))};
      }();
      return kSingleton;
    } catch (std::exception const &err) {
      std::cerr << std::format(
          "{}:{}:{}: caught exception while constructing INTST singleton: "
          "{}\n",
          __FILE__, __LINE__, __PRETTY_FUNCTION__, err.what());
      std::terminate();
    }
  }
};
