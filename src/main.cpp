#include <iostream>
#include <spdlog/spdlog.h>

void fun()
{
    std::cout << "hello cmake" << std::endl;
}

int main(int, char **)
{
    SPDLOG_INFO("hello world");
    
    return 0;
}