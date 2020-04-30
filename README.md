# liblogger
C++17 logging facility.
Header-only library for logging.

User can either retrieve a singleton instance which logs to `stdout`/`stderr`, or construct a new instance of logger backed by
arbitrary `std::ostream` object(s) (files, sockets, etc...).

Output lines are prefixed with ISO timestamps and suffixed `+<duration>`,[debug](https://www.npmjs.com/package/debug)-style.

## API
* `Logger(std::ostream &outStream, std::ostream &errStream)`: constructor outStream and errStream defaults to `stdin` and `stderr`
* `Logger::Info`, `Logger::Warn`, `Logger:Error`, `Logger:Debug`, `Logger:Trace`: different output levels with indicative formatting
* `setUnifiedOutput(bool)`: use one stream for all IO
* `setVerbosity(level)`: with `level` being one of `Verbosity::{QUIET, ERR, WARN, INFO, DEBUG, TRACE}`
* `static Logger::get()`: retrieve global default-constructred singleton

## Sample usage
```c++
#include <ctime>
#include <cstdlib>
#include <algorithm>

#include "logutil.h"

int main(int argc, char *argv[]) {
	std::srand(std::time(nullptr));
	auto &logger = Logger::get(); // default logger

	struct randInts {
		randInts() = default;
		randInts(ssize_t n): n_{n} {}
		bool operator ==(randInts const &rhs) { return rhs.n_ == n_; }
		bool operator !=(randInts const &rhs) { return !(*this == rhs); }
		randInts &operator++() { --n_; return *this; }
		int operator*() { return std::rand(); }
		private: ssize_t n_{};
	};

	std::for_each(randInts(5), randInts(), [&](int i) { // by-ref capture for 'logger'
			INFO("random int: {}", i);
			});
}
```
