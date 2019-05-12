#include "source.h"

#include <iostream>

namespace prj {
Source::Source(int g) { std::cout << "in constructor: " << g << std::endl; }
Source::~Source() {}
}  // namespace prj