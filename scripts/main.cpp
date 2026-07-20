#include <iostream>
#include <thread>
#include "shared_state.h"

bool initSpacemouse();
void spacemouseThread();
void closeSpacemouse();

// void controlThread(const std::string& hostname);
void printState(const std::string& hostname);

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }

    if (!initSpacemouse()) { // initialize spacemouse
        std::cerr << "Failed to lock HID descriptors. Exiting." << std::endl;
        return -1;
    }

    std::cout << "Starting spacemouse thread" << std::endl;
    std::thread mouseThread(spacemouseThread); // main polling thread to read spacemouse input

    std::cout << "Starting printstate thread" << std::endl;
    printState(argv[1]);
    // run franka ctrl loop on main thread
    // controlThread(argv[1]);

    mouseThread.detach();
    closeSpacemouse();
    
    return 0;
}