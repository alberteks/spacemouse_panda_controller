cls
gcc -02 -DNDEBUG main.cpp shared_state.cpp spacemouse.cpp arm_ctrl.cpp \
    -o spacemouse_teleop.exe \
    -I..\assets\hidapi\include \
    -I..\..\libfranka\include\franka \
    
    -L..\assets\hidapi\x64 \
    -lhidapi \
    -lfranka \
    -lpthread \
.\spacemouse_teleop.exe