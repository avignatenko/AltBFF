#pragma once

#include <functional>

class Runner
{
public:
    virtual bool run(std::function<void()> func) = 0;

    thread_local static Runner* threadInstance;
};
