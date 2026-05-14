#include <fcntl.h>
#include <unistd.h>
#include <cmath>
#include <iostream>
#include "Pipe.hpp"

static constexpr float D = M_PI / 180.0f; // deg to rad

static void send(int fd,
                 float rx0, float ry0,
                 float rx1, float ry1,
                 float rx2, float ry2,
                 int ms) {
    ArmCommand cmd;
    cmd.joints[0] = { rx0*D, ry0*D };
    cmd.joints[1] = { rx1*D, ry1*D };
    cmd.joints[2] = { rx2*D, ry2*D };
    write(fd, &cmd, sizeof(cmd));
    usleep(ms * 1000);
}

int main() {
    int fd = open(PIPE_PATH, O_WRONLY);
    if (fd < 0) {
        std::cerr << "Cannot open pipe. Is robot-arm running?\n";
        return 1;
    }
    std::cout << "Controller connected\n";
    while (true){
    //        j0_rx   j0_ry  |  j1_rx   j1_ry  |  j2_rx   j2_ry  |  ms
    send(fd,   0.0f,   0.0f,     0.0f,   0.0f,     0.0f,   0.0f,   1000); 
    send(fd,   0.0f,   0.0f,    90.0f,   0.0f,    90.0f,   0.0f,   1000); 
    send(fd,   0.0f, 180.0f,    90.0f,   0.0f,    90.0f,   0.0f,   1000); 
    send(fd,   0.0f,  90.0f,    90.0f,   0.0f,    90.0f,   0.0f,   1000); 
    send(fd,  45.0f,  90.0f,    90.0f,   0.0f,    45.0f,   0.0f,   1000); 
    send(fd,   0.0f,  90.0f,    90.0f,   0.0f,    90.0f,   0.0f,   1000); 
    send(fd,   0.0f,   0.0f,    90.0f,   0.0f,    90.0f,   0.0f,   1000); 
    send(fd,   0.0f,  90.0f,    45.0f,  90.0f,    90.0f,   0.0f,   1000); 
    send(fd,   0.0f,  90.0f,     0.0f,  90.0f,    90.0f,   0.0f,   1000); 
    send(fd,   0.0f,   0.0f,     0.0f,   0.0f,    90.0f,   0.0f,   1000); 
    send(fd,  45.0f,   0.0f,    90.0f,   0.0f,    45.0f,   0.0f,   1000); 
    send(fd,  45.0f,   0.0f,    90.0f,   0.0f,    45.0f, 360.0f,   1000); 
    send(fd,  45.0f,   0.0f,    90.0f, 360.0f,    45.0f, 360.0f,   1000);  
    send(fd,  45.0f,   0.0f,    90.0f, 360.0f,    45.0f,   0.0f,   1000);  
    send(fd,  45.0f,   0.0f,    90.0f,   0.0f,    45.0f,   0.0f,   1000);  
   
    }
    close(fd);
    std::cout << "Séquence terminée\n";
    return 0;
}