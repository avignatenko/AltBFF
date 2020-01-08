
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
            while (!stopped_)
            {
                std::this_thread::sleep_for(delay_);

                std::promise<bool> result;

                bool runOk = d_.run([this, &result] {
                    result.set_value(callback_());
                });

                // fixme: take care of time spent for waiting
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
