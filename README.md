# liblogger
C++17 logging facility.
Header-only library for logging.

User can either retrieve a singleton instance which logs to `stdout`/`stderr`, or construct a new instance of logger backed by
arbitrary `std::ostream` object(s) (files, sockets, etc...).

Output lines are prefixed with ISO timestamps and suffixed `+<duration>`,[debug](https://www.npmjs.com/package/debug)-style.

![Sample Session](sample-session.svg)

## API
* `Logger(std::ostream &outStream, std::ostream &errStream)`: constructor outStream and errStream defaults to `stdin` and `stderr`
* `Logger::Info`, `Logger::Warn`, `Logger:Error`, `Logger:Debug`, `Logger:Trace`: different output levels with indicative formatting
* `setUnifiedOutput(bool)`: use one stream for all IO
* `setVerbosity(level)`: with `level` being one of `Verbosity::{QUIET, ERR, WARN, INFO, DEBUG, TRACE}`
* `static Logger::get()`: retrieve global default-constructred singleton

## Sample usage
```c++
#include <fstream>
#include <chrono>
#include <ctime>
#include <cstdlib>
#include <algorithm>

#include "logutil.h"

int main(int argc, char *argv[]) {
	std::srand(std::time(nullptr));
	auto &logger = Logger::get(); // default logger

	auto hd_logfile = std::ofstream{std::to_string(std::time(nullptr)) + ".txt"};
	auto hd_logger = Logger{hd_logfile, hd_logfile};
	hd_logger.setVerbosity(Logger::Verbosity::TRACE);

	struct randInts {
		randInts() = default;
		explicit randInts(ssize_t n): n_{n} {}
		bool operator ==(randInts const &rhs) const { return rhs.n_ == n_; }
		bool operator !=(randInts const &rhs) const { return !(*this == rhs); }
		randInts &operator++() { --n_; return *this; }
		int operator*() const { return std::rand() * ((std::rand() % 2) ? -1 : 1); }
		private: ssize_t n_{};
	};

	std::for_each(randInts(5), randInts(), [&](int i) { // by-ref capture for 'logger'
			// raw lines, console.debug alike, does not support fmt::format syntax
			hd_logger.Trace("random integer:", i);
			// using fmt::format
			hd_logger.Trace(fmt::format("hig-res timestamp: {}ns", std::chrono::high_resolution_clock::now().time_since_epoch().count()));

		       	// managed logger, pretty prints
			DEBUG("random int squared: {}", i*i);
			// will not be output to stderr, since default logger's verbosity is DEBUG
			TRACE("random int: {}", i);
			});
}
```
