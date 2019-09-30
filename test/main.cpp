#include <iostream>
#include "source.h"

int main(int argc, char** argv) {
    std::cout << "hello world\n";

    gui::PangolinGui* kj = new gui::PangolinGui();
    kj->Run();

    return 0;
}