#pragma once

#include <memory>
#include <queue>

// ObjectPool is to avoid frequest alloc/dealloc when sending in async way
template <class T>
class ObjectPool
{
public:
    ObjectPool(size_t defSize = 1)
    {
        for (int i = 0; i < defSize; ++i) objects_.push(std::make_shared<T>());
    }

    std::shared_ptr<T> aquire()
    {
        if (!objects_.empty())
        {
            std::shared_ptr<T> obj = std::move(objects_.front());
            objects_.pop();
            return obj;
        }

        return std::make_shared<T>();
    }
    void release(std::shared_ptr<T> object) { objects_.push(std::move(object)); }

    size_t size() const { return objects_.size(); }

private:
    std::queue<std::shared_ptr<T>> objects_;
};
