#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include "tercom/radar/position.h"
#include "tercom/tercom.h"
#include "tercom/trajectory/target.h"

volatile sig_atomic_t g_signal = 0;

void signalHandler(int signal) {
    g_signal = signal;
}

TERCOM::Radar::Position getRadarData() {
    return TERCOM::Radar::Position(0, 0, 0);
}

int main() {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::string path = "";

    TERCOM::Radar::Position position  = TERCOM::Radar::Position(0, 0, 0);
    TERCOM::Trajectory::Target target = TERCOM::Trajectory::Target(0, 0, 0);

    TERCOM::GuidanceSystem gs = TERCOM::GuidanceSystem(path, position, target);

    while (1) {
        if (g_signal != 0) {
            std::cout << "Signal " << g_signal << " received. Exiting." << std::endl;
            break;
        }

        auto pos = getRadarData();
        auto cor = gs.CalculateCorrections(pos);

        std::cout << "Correction" << std::endl;
        std::cout << "Roll:" << cor.Roll() << std::endl;
        std::cout << "Pitch:" << cor.Pitch() << std::endl;
        std::cout << "Yaw:" << cor.Yaw() << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
