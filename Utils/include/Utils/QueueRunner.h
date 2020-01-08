#pragma once

#include "DispatchQueue.h"
#include "IRunner.h"

class QueueRunner : public Runner
{
public:
    QueueRunner() { threadInstance = this; }
    ~QueueRunner() { threadInstance = nullptr; }

    virtual bool run(std::function<void()> func)
    {
        if (stopped()) return false;
        m_dispatchQueue.put(func);
        return true;
    }

    void run()
    {
        while (!m_done)
        {
            auto functor = m_dispatchQueue.take();
            if (functor)
                functor();
            else
                m_done = true;
        }

        // dispatch remaing functions
        while (auto functor = m_dispatchQueue.takeNonBlocking()) functor();
    }

    void stop() { run(nullptr); }
    bool stopped() const { return m_done; }

private:
    DispatchQueue m_dispatchQueue;
    std::atomic<bool> m_done = false;
};