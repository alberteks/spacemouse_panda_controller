#pragma once
#include <mutex>

struct SpacemouseInput {
    int x = 0; 
    int y = 0;
    int z = 0; 
    int roll = 0;
    int pitch = 0; 
    int yaw = 0; 
};

//global instances accessible by both spacemouse controller and robot controller
extern SpacemouseInput g_mouseData;
extern std::mutex g_mouseMutex; 