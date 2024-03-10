#include "logutil.h"
extern "C" void libmain() {
  auto &logger = Logger::get();
  TRACE(
      "from example dylib: should inherit verbosity level trace from main "
      "shared obj");
}
