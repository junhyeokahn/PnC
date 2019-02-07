#pragma once

#include <zmq.hpp> // https://github.com/zeromq/cppzmq

namespace myUtils
{
    static std::string s_recv (zmq::socket_t & socket) {

        zmq::message_t message;
        socket.recv(&message);

        return std::string(static_cast<char*>(message.data()), message.size());
    }

    //  Convert string to 0MQ string and send to socket
    static bool s_send (zmq::socket_t & socket, const std::string & string) {

        zmq::message_t message(string.size());
        memcpy (message.data(), string.data(), string.size());

        bool rc = socket.send (message);
        return (rc);
    }
} /* myUtils */
