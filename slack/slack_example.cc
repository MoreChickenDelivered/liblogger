#include <string>
#include <chrono>

#include <nlohmann/json.hpp>

#include "slack.h"

int main(int argc, char *argv[]) {
	std::string boticon, botlink, botname;

	if (!getenv("SLACK_WEBHOOK"))
		throw std::runtime_error{"need SLACK_WEBHOOK in env"};
	if (auto const evar = getenv("SLACK_BOTICON"); evar)
		boticon = evar;
	if (auto const evar = getenv("SLACK_BOTLINK"); evar)
		botlink = evar;
	if (auto const evar = getenv("SLACK_BOTNAME"); evar)
		botname = evar;

	auto const slack_cfg = nlohmann::json{
		{ "webhook", getenv("SLACK_WEBHOOK") },
			{ "boticon", boticon },
			{ "botlink", botlink },
			{ "botname", botname },
	};

	auto slack = Slack::get(&slack_cfg);

	using Severity = Slack::Severity;

	slack->send("Testing INFO", "Pertinent Information....", Severity::INFO, true);
	std::this_thread::sleep_for(std::chrono::seconds(2));

	slack->send("Testing ERROR", "Pertinent Information....", Severity::ERROR, true);

	std::this_thread::sleep_for(std::chrono::seconds(2));
	slack->send("Testing WARN", "Pertinent Information....", Severity::WARN);

	std::this_thread::sleep_for(std::chrono::seconds(2));
	slack->send("Testing DEBUG", "Pertinent Information....", Severity::DEBUG);

	std::this_thread::sleep_for(std::chrono::seconds(3));
}
