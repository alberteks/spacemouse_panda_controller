cls
gcc main.cpp shared_state.cpp spacemouse.cpp arm_ctrl.cpp \
    -o spacemouse_teleop.exe \
    -I..\assets\hidapi\include \
    -L..\assets\hidapi\x64 \
    -lhidapi
    -lpthread
.\spacemouse_teleop.exe