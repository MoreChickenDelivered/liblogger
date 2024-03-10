/**
 *  Elias Benali <ebenali@tradeterminal.io>
 *  C++17 header-only sync/async slack messaging library (webhook based,
 * markdown) Offers a singleton 'default_messenger': Slack::get() or
 * Slack::SlackMessenger::get() Uses Linux-specific /proc/stat Dependencies:
 * cURL, hhinnant date, nlohmann::json 2020-12-07
 */
#pragma once

#include <curl/curl.h>
#include <curl/easy.h>
#include <date/date.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <format>
#include <iostream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>
#include <vector>

namespace Slack {
enum class Severity {
  kInfo,
  kError,
  kWarn,
  kDebug,
};

struct SlackMessenger {
  SlackMessenger(std::string webhook, std::string botname, std::string botlink,
                 std::string boticon)
      : slack_webhook_{std::move(webhook)},
        slack_botname_{std::move(botname)},
        slack_botlink_{std::move(botlink)},
        slack_boticon_{std::move(boticon)} {
    if (slack_webhook_.empty())
      throw std::runtime_error{
          std::format("{}: webhook URL can not be empty", __func__)};
    setup_slack();
  }

  explicit SlackMessenger(nlohmann::json const &cfg)
      : SlackMessenger(cfg.count("webhook") ? cfg["webhook"].get<std::string>()
                                            : std::string{},
                       cfg.count("botname") ? cfg["botname"].get<std::string>()
                                            : std::string{},
                       cfg.count("botlink") ? cfg["botlink"].get<std::string>()
                                            : std::string{},
                       cfg.count("boticon") ? cfg["boticon"].get<std::string>()
                                            : std::string{}) {
    if (cfg.count("severity")) {
      auto const &requested_severity =
          cfg["severity"].get_ref<std::string const &>();
    }
  }

  ~SlackMessenger() {
    std::unique_lock lock{slack_queue_mtx_};
    thread_bail_out_ = true;
    lock.unlock();
    slack_queue_cnd_.notify_one();
    slack_dispatch_thread_.join();
  }

  /**
   *  async operation by default
   *  'text' is markdown formatted
   */
  void send(std::string const &title, std::string const &body,
            Severity level = Severity::kInfo, bool sync = false,
            bool is_warning = false) {
    if (default_severity_ < level) return;
    while (!thread_initialized_)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    std::unique_lock<std::mutex> lock;
    if (last_sending_time_.contains(title) &&
        (std::chrono::system_clock::now() - last_sending_time_[title]) <
            std::chrono::milliseconds(100)) {
      return;
    }

    last_sending_time_[title] = std::chrono::system_clock::now();
    if (sync) lock = std::unique_lock<std::mutex>{slack_queue_mtx_};
    enqueue_slack(
        nlohmann::json{
            {"attachments",
             std::vector<nlohmann::json>{
                 {
                     {"mrkdwn_in", {"text"}},
                     {"color", level == Severity::kInfo    ? "#beef33"
                               : level == Severity::kError ? "#ff0000"
                               : level == Severity::kWarn  ? "#eaaa22"
                                                           : "#888888"},
                     {"author_name", slack_botname_.empty()
                                         ? "<unspecific-bot-name>"
                                         : slack_botname_},
                     {"author_link",
                      slack_botlink_.empty() ? "slack.com" : slack_botlink_},
                     {"author_icon",
                      slack_boticon_.empty()
                          ? "https://img.favpng.com/15/0/15/"
                            "mars-exploration-rover-mars-rover-clip-art-png-"
                            "favpng-Qhmu0a8hkuHhvVcpMjU0C0BQ0.jpg"
                          : slack_boticon_},
                     {"title", title},
                     {"text", body},
                     {"footer",
                      std::format("pid {}; {}{}", getpid(),
                                  SlackMessenger::proc_stat(),
                                  optional_footer_ ? optional_footer_()
                                                   : std::string{})},
                     {"ts",
                      std::chrono::duration_cast<std::chrono::seconds>(
                          std::chrono::system_clock::now().time_since_epoch())
                          .count()},
                 },
             }},
            {"slack_webhook",
             is_warning ? slack_warning_webhook_ : slack_webhook_},
        },
        sync);
    if (sync) {
      lock.unlock();
      slack_queue_cnd_.notify_one();
      lock.lock();
      if (!slack_queue_.empty())
        slack_empty_cnd_.wait(lock, [&] { return slack_queue_.empty(); });
    }
  }

  void set_footer_callback(std::function<std::string(void)> callback) {
    optional_footer_ = std::move(callback);
  }

  /**
   *  singleton interface
   *  @config has to be only passed first time around
   */
  static std::shared_ptr<SlackMessenger> get(nlohmann::json const *config) {
    static std::shared_ptr<SlackMessenger> singleton = nullptr;
    if (!singleton && config)
      singleton = std::make_shared<SlackMessenger>(*config);
    return singleton;
  }

 private:
  void setup_slack() {
    slack_dispatch_thread_ = std::thread{[&]() {
      // LOG(INFO) << "Slack message dispatch initialized";
      for (;;) {
        std::unique_lock lock{slack_queue_mtx_};

        thread_initialized_ = true;

        slack_queue_cnd_.wait(
            lock, [&] { return thread_bail_out_ || !slack_queue_.empty(); });
        if (thread_bail_out_) break;

        // auto t_0 = std::chrono::system_clock::now();
        while (!slack_queue_.empty()) {
          auto &msg = slack_queue_.front();
          send_slack(std::move(msg));
          slack_queue_.erase(std::begin(slack_queue_));
          // if (std::chrono::system_clock::now() - t_0 >
          // std::chrono::seconds(1)) 	break;
        }

        lock.unlock();
        slack_empty_cnd_.notify_all();
      }
    }};
  }

  static std::string proc_stat() {
    struct stat statbuf;
    if (stat("/proc/self/statm", &statbuf) != 0) return "";
    FILE *fptr = fopen("/proc/self/statm", "r");

    uint64_t f_size;
    uint64_t f_resident;
    uint64_t f_shared;
    uint64_t f_text;
    uint64_t f_lib;
    uint64_t f_data;
    uint64_t f_dt;

    (void)fscanf(fptr, "%lu %lu %lu %lu %lu %lu %lu", &f_size, &f_resident,
                 &f_shared, &f_text, &f_lib, &f_data, &f_dt);

    const auto pg_sz = sysconf(_SC_PAGESIZE);
    constexpr auto kOneMegabyte = static_cast<double>(1 << 20);

    return std::format("VM: {:.2f}M; RSS: {:.2f}M",
                       static_cast<double>(f_size * pg_sz) / kOneMegabyte,
                       static_cast<double>(f_resident * pg_sz) / kOneMegabyte);
  }

  void enqueue_slack(nlohmann::json msg, bool locked = false) {
    // std::unique_lock lk{slack_queue_mtx_};
    std::unique_lock<std::mutex> lock;
    if (!locked) lock = std::unique_lock<std::mutex>{slack_queue_mtx_};
    slack_queue_.emplace_back(std::move(msg));
    if (!locked) {
      lock.unlock();
      slack_queue_cnd_.notify_one();
    }
  }

  static void send_slack(nlohmann::json msg) {
    auto *curl = curl_easy_init();
    if (!curl) throw std::runtime_error{"curl init failure (curl_easy_init)"};
    struct curl_slist *hdrs = nullptr;
    auto const slack_webhook = msg["slack_webhook"].get<std::string>();
    msg.erase("slack_webhook");
    auto const serialized = msg.dump();
    hdrs = curl_slist_append(hdrs, "Content-Type: application/json");
    curl_easy_setopt(curl, CURLOPT_URL, slack_webhook.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, serialized.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, serialized.size());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, hdrs);
    if (auto res = curl_easy_perform(curl); res != CURLE_OK) {
      std::cerr << std::format("curl HTTP req errored out: {}\n",
                               curl_easy_strerror(res));
      std::cerr.flush();
    }
    curl_slist_free_all(hdrs);
    curl_easy_cleanup(curl);
  }

  std::vector<nlohmann::json> slack_queue_;

  std::condition_variable slack_queue_cnd_;
  std::condition_variable slack_empty_cnd_;

  std::atomic<bool> thread_bail_out_ = false;
  std::atomic<bool> thread_initialized_ = false;

  std::mutex slack_queue_mtx_;

  std::string slack_webhook_;
  std::string slack_botname_;
  std::string slack_botlink_;

  const std::string slack_warning_webhook_ =
      "https://hooks.slack.com/services/TMHFVT43G/B0567ME6E9W/"
      "Hb1FHrJ4rFFS7LlkrALRlFN2";  // hft-binance-dev-warn

  std::string slack_boticon_;
  std::thread slack_dispatch_thread_;

  std::function<std::string()> optional_footer_ = nullptr;

  std::map<std::string, std::chrono::system_clock::time_point>
      last_sending_time_;

  Severity default_severity_ = Severity::kDebug;
};

static std::shared_ptr<SlackMessenger> get(nlohmann::json const *config) {
  return SlackMessenger::get(config);
}

}  // namespace Slack
