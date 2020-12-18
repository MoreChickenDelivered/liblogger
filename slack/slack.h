/**
 *  Elias Benali <ebenali@tradeterminal.io>
 *  C++17 header-only sync/async slack messaging library (webhook based, markdown)
 *  Offers a singleton 'default_messenger': Slack::get() or Slack::SlackMessenger::get()
 *  Uses Linux-specific /proc/stat
 *  Dependencies: cURL, hhinnant date, libfmt, nlohmann::json
 *  2020-12-07
 */
#pragma once

#include <iostream>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <string>
#include <thread>
#include <mutex>
#include <vector>
#include <unistd.h>
#include <sys/stat.h>
#include <nlohmann/json.hpp>
#include <curl/curl.h>
#include <curl/easy.h>
#include <fmt/format.h>
#include <date/date.h>

namespace Slack {
	enum class Severity {
		INFO,
		ERROR,
		WARN,
		DEBUG,
	};

	struct SlackMessenger {
		SlackMessenger(std::string webhook, std::string botname, std::string botlink, std::string boticon):
			slack_webhook_{std::move(webhook)},
			slack_botname_{std::move(botname)},
			slack_botlink_{std::move(botlink)},
			slack_boticon_{std::move(boticon)} {
				if (slack_webhook_.empty())
					throw std::runtime_error{fmt::format("{}: webhook URL can not be empty", __func__)};
				setup_slack();
			}

		explicit SlackMessenger(nlohmann::json const &cfg):
			SlackMessenger(
				cfg.count("webhook") ? cfg["webhook"].get<std::string>() : std::string{},
				cfg.count("botname") ? cfg["botname"].get<std::string>() : std::string{},
				cfg.count("botlink") ? cfg["botlink"].get<std::string>() : std::string{},
				cfg.count("boticon") ? cfg["boticon"].get<std::string>() : std::string{}) {}

		~SlackMessenger() {
			std::unique_lock lk{slack_queue_mtx_};
			thread_bail_out_ = true;
			lk.unlock();
			slack_queue_cnd_.notify_one();
			slack_dispatch_thread_.join();
		}

		/**
		 *  async operation by default
		 *  'text' is markdown formatted
		 */
		void send(std::string title, std::string body, Severity level = Severity::INFO, bool sync=false) {
			std::unique_lock<std::mutex> lk;
			if (sync)
				lk = std::unique_lock<std::mutex>{slack_queue_mtx_};
			enqueue_slack(nlohmann::json{
				{ "attachments", 
					std::vector<nlohmann::json>{
						{
							{ "mrkdwn_in", { "text" } },
							{ "color", level == Severity::INFO ? "#beef33" : level == Severity::ERROR ? "#ff0000" : level == Severity::WARN ? "#eaaa22" : "#888888" },
							{ "author_name", slack_botname_.empty() ? "<unspecific-bot-name>" : slack_botname_ },
							{ "author_link", slack_botlink_.empty() ? "slack.com" : slack_botlink_ },
							{ "author_icon", slack_boticon_.empty() ? "https://img.favpng.com/15/0/15/mars-exploration-rover-mars-rover-clip-art-png-favpng-Qhmu0a8hkuHhvVcpMjU0C0BQ0.jpg" : slack_boticon_ },
							{ "title", title },
							{ "text", body },
							{ "footer", fmt::format("pid {}; {}{}", getpid(), SlackMessenger::proc_stat(), optional_footer_ ? optional_footer_() : std::string{}) },
							{ "ts", std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count() },
						},
					}
				},
			}, sync);
			if (sync) {
				lk.unlock();
				slack_queue_cnd_.notify_one();
				lk.lock();
				if (!slack_queue_.empty())
					slack_empty_cnd_.wait(lk, [&]{ return slack_queue_.empty(); });
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
			slack_dispatch_thread_ = std::thread{
				[&]() {
					//LOG(INFO) << "Slack message dispatch initialized";
					for (;;) {
						std::unique_lock lk{slack_queue_mtx_};

						slack_queue_cnd_.wait(lk, [&] {return thread_bail_out_ || !slack_queue_.empty();});
						if (thread_bail_out_)
							break;

						//auto t_0 = std::chrono::system_clock::now();
						while (!slack_queue_.empty()) {
							auto &msg = slack_queue_.front();
							send_slack(std::move(msg));
							slack_queue_.erase(std::begin(slack_queue_));
							//if (std::chrono::system_clock::now() - t_0 > std::chrono::seconds(1))
							//	break;
						}

						lk.unlock();
						slack_empty_cnd_.notify_all();
					}
				}
			};
		}

		static std::string proc_stat() {
			struct stat st;
			if (stat("/proc/self/statm", &st) != 0)
				return "";
			FILE *fp = fopen("/proc/self/statm", "r");
			unsigned long f_size, f_resident, f_shared, f_text, f_lib, f_data, f_dt;
			fscanf(fp, "%lu %lu %lu %lu %lu %lu %lu", &f_size, &f_resident, &f_shared, &f_text, &f_lib, &f_data, &f_dt);
			const auto pg_sz = sysconf(_SC_PAGESIZE);
			return fmt::format("VM: {:.2f}M; RSS: {:.2f}M", f_size * pg_sz / double(1<<20), f_resident * pg_sz / double(1<<20));
		}

		void enqueue_slack(nlohmann::json msg, bool locked=false) {
			//std::unique_lock lk{slack_queue_mtx_};
			std::unique_lock<std::mutex> lk;
			if (!locked)
				lk = std::unique_lock<std::mutex>{slack_queue_mtx_};
			slack_queue_.emplace_back(std::move(msg));
			if (!locked) {
				lk.unlock();
				slack_queue_cnd_.notify_one();
			}
		}

		void send_slack(nlohmann::json msg) {
			auto cl = curl_easy_init();
			if (!cl)
				throw std::runtime_error{"curl init failure (curl_easy_init)"};
			auto const serialized = msg.dump();
			struct curl_slist *hdrs = nullptr;
			hdrs = curl_slist_append(hdrs, "Content-Type: application/json");
			curl_easy_setopt(cl, CURLOPT_URL, slack_webhook_.c_str());
			curl_easy_setopt(cl, CURLOPT_POSTFIELDS, serialized.c_str());
			curl_easy_setopt(cl, CURLOPT_POSTFIELDSIZE, serialized.size());
			curl_easy_setopt(cl, CURLOPT_HTTPHEADER, hdrs);
			if (auto res = curl_easy_perform(cl); res != CURLE_OK) {
				std::cerr << fmt::format("curl HTTP req errored out: {}\n", curl_easy_strerror(res));
				std::cerr.flush();
			}
			curl_slist_free_all(hdrs);
			curl_easy_cleanup(cl);
		}

		std::vector<nlohmann::json> slack_queue_;
		std::condition_variable slack_queue_cnd_;
		std::condition_variable slack_empty_cnd_;
		std::atomic<bool> thread_bail_out_;
		std::mutex slack_queue_mtx_;
		std::string slack_webhook_;
		std::string slack_botname_;
		std::string slack_botlink_;
		std::string slack_boticon_;
		std::thread slack_dispatch_thread_;
		std::function<std::string()> optional_footer_ = nullptr;
	};

	static std::shared_ptr<SlackMessenger> get(nlohmann::json const *config) {
		return SlackMessenger::get(config);
	}

} // namespace Slack
