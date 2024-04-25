#include "logutil.h"

extern "C" void libmain() {
  auto &logger = Logger::get();
  WARN("vanilla C/C++ is not sufficient not handle the dlopen use case");
}
