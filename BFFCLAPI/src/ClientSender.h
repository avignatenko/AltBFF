#pragma once

#include "CLStructures.h"

#include <Utils/PeriodicTimer.h>

#include <asio.hpp>
#include <asio/io_context.hpp>

#include <queue>

namespace bffcl
{

class ClientSender
{
public:
    ClientSender(asio::io_context& io, asio::ip::udp::socket& socket, const std::string& addressRemote, int portRemote,
                 double freq);
    ~ClientSender();

    CLInput& lockInput();

private:
    void send();

private:
    asio::io_context& io_;
    asio::ip::udp::socket& socket_;
    asio::ip::udp::endpoint endpointRemote_;

    PeriodicTimer sendTimer_;

    unsigned int packetId_ = 1;

    CLInput currentInput_;

    // ObjectPool is to avoid frequest alloc/dealloc when sending in async way
    template <class T>
    class ObjectPool
    {
    public:
        ObjectPool(size_t defSize = 1)
        {
            for (int i = 0; i < defSize; ++i) objects_.push(std::make_shared<T>());
        }

        std::shared_ptr<T> aquire(const T& data)
        {
            if (!objects_.empty())
            {
                std::shared_ptr<T> obj = std::move(objects_.front());
                objects_.pop();
                *obj = data;
                return obj;
            }

            return std::make_shared<T>(data);
        }
        void release(std::shared_ptr<T> object) { objects_.push(std::move(object)); }

        size_t size() const { return objects_.size(); }

    private:
        std::queue<std::shared_ptr<T>> objects_;
    };

    ObjectPool<CLInput> inputPool_;
};

}  // namespace bffcl
