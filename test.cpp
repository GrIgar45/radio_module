//
// Created by dev on 2/8/18.
//
#include <unistd.h>
#include <thread>
#include <iostream>


int main(int argc, char *argv[]) {
    std::chrono::high_resolution_clock::time_point start, final;
    std::chrono::high_resolution_clock::duration timeSpend {};
    start = std::chrono::high_resolution_clock::now();
    sleep(1);
    final = std::chrono::high_resolution_clock::now();

    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(final - start).count();
    return 0;
}

