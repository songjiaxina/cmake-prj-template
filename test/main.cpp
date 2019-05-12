#include <iostream>
#include "source.h"

int main(int argc, char** argv) {
    std::cout << "hello world\n";
    prj::Source* source = new prj::Source(10);
    delete source;
    return 0;
}