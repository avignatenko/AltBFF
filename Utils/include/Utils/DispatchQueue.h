#pragma once

#include <functional>
#include <mutex>
#include <queue>

class DispatchQueue
{
public:
    using Operation = std::function<void()>;

    void put(Operation op)
    {
        std::lock_guard<std::mutex> guard(m_qlock);
        m_opsQueue.push(op);
        m_empty.notify_one();
    }

    Operation take()
    {
        std::unique_lock<std::mutex> lock(m_qlock);
        m_empty.wait(lock, [&] { return !m_opsQueue.empty(); });

        Operation op = m_opsQueue.front();
        m_opsQueue.pop();
        return op;
    }

    Operation takeNonBlocking()
    {
        std::lock_guard<std::mutex> guard(m_qlock);
        if (m_opsQueue.empty()) return nullptr;

        Operation op = m_opsQueue.front();
        m_opsQueue.pop();
        return op;
    }

private:
    mutable std::mutex m_qlock;
    std::queue<Operation> m_opsQueue;
    std::condition_variable m_empty;
};
