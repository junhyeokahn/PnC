#include <ADE/DracoWrapper.hpp>
#include <iostream>
#include <thread>

void StartControls(DracoWrapper& w) {
    bool live = true;
    while (live) {
        char nextInput;
        std::cout << "Enter a move command or c to exit\n";
        std::cin >> nextInput;
        switch (nextInput) {
            case 'w':
                std::cout << "Walking!\n";
                w.SetWalkCommand();
                break;
            case 'c':
                std::cout << "Exiting!\n";
                live = false;
                break;
            default:
                break;
        }
    }
}

int main(int argc, char** argv) {
    DracoWrapper w;
    w.Initialize();
    StartControls(w);
    w.Shutdown();
    return 0;
}