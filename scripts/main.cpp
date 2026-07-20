#include <iostream>
#include <thread>
#include "shared_state.h"

void spacemouseThread();
void controlThread(const std::string& hostname);

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }

    std::thread mouseThread(spacemouseThread);

    // run franka ctrl loop on main thread
    controlThread(argv[1]);
    mouseThread.detach();
    
    return 0;
}