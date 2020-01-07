
#include <memory>
#include <thread>
#include "IRunner.h"

class Timer
{
public:
    Timer(std::chrono::milliseconds delay, Runner& d, std::function<void()> callback)
        : delay_(delay), callback_(callback), d_(d)
    {
    }

    void singleShot()
    {
        stop();

        m_timer = std::make_unique<std::thread>([this] {
            std::this_thread::sleep_for(delay_);
            d_.run([this] { callback_(); });
        });
    }

    void stop()
    {
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
    std::function<void()> callback_;
    Runner& d_;
};
