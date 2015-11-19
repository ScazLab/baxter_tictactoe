#include <iostream>
#include <cstdio>
#include "src/utils/BufferToggle.h"

int main()
{
    BufferToggle bt;

    char c;
    bt.off();
    std::cout << "Processes next instruction as soon as you type a character (no enter)" << std::endl;
    c=std::getchar();
    std::cout << "1." << c <<std::endl;

    c=std::getchar();
    std::cout << "2." << c <<std::endl;

    bt.on();
    std::cout << "Waits for you to press enter before proceeding to the next instruction" << std::endl;
    c=std::getchar();
    std::cout << "3." << c <<std::endl;

    return 0;
}
