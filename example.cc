// (C) 2019 TT.io
// Author: Elias Benali <ebenali@tradeterminal.io>
// Created by deb on 11/23/19.
//

#include <ctime>
#include <cstdlib>
#include <algorithm>

#include "logutil.h"

int main(int argc, char *argv[]) {
	std::srand(std::time(nullptr));
	auto &logger = Logger::get(); // default logger

	struct randInts {
		randInts() = default;
		explicit randInts(ssize_t n): n_{n} {}
		bool operator ==(randInts const &rhs) const { return rhs.n_ == n_; }
		bool operator !=(randInts const &rhs) const { return !(*this == rhs); }
		randInts &operator++() { --n_; return *this; }
		int operator*() const { return std::rand(); }
		private: ssize_t n_{};
	};

	std::for_each(randInts(5), randInts(), [&](int i) { // by-ref capture for 'logger'
			INFO("random int: {}", i);
			});
}
