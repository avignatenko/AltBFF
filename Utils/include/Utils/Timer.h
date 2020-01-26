
#include <memory>
#include <thread>
#include "IRunner.h"

class Timer
{
public:
    Timer(std::chrono::milliseconds delay, Runner& d, std::function<bool()> callback)
        : delay_(delay), callback_(callback), d_(d)
    {
    }

    void start()
    {
 
        m_timer = std::make_unique<std::thread>([this] {
            std::chrono::time_point<std::chrono::steady_clock> targetTime = std::chrono::steady_clock::now();
            while (!stopped_)
            {
                targetTime += delay_;
                std::this_thread::sleep_until(targetTime);

                std::promise<bool> result;

                bool runOk = d_.run([this, &result] {
                    result.set_value(callback_());
                });

                stopped_ = runOk ? result.get_future().get() : true;
            }
        });
    }

    void stop()
    {
        stopped_ = true;
        if (m_timer)
        {
            m_timer->join();
            m_timer.reset();
        }
    }

    ~Timer() { stop(); }

private:
    std::unique_ptr<std::thread> m_timer;

    std::chrono::milliseconds delay_;
    std::function<bool()> callback_;
    std::atomic_bool stopped_ = false;
    Runner& d_;
};
