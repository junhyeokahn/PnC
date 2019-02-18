#pragma once

#include <zmq.hpp>  // https://github.com/zeromq/cppzmq

namespace myUtils {
static std::string StringRecv(zmq::socket_t& socket) {
    zmq::message_t message;
    socket.recv(&message);

    return std::string(static_cast<char*>(message.data()), message.size());
}

//  Convert string to 0MQ string and send to socket
static bool StringSend(zmq::socket_t& socket, const std::string& string) {
    zmq::message_t message(string.size());
    memcpy(message.data(), string.data(), string.size());

    bool rc = socket.send(message);
    return (rc);
}

// cpp send hello.
// when python gets connected and recv hello, then req world
// when cpp get req, rep dummy and break
static void PairAndSync(zmq::socket_t& pub_socket, zmq::socket_t& rep_socket,
                        int num_subscriber) {
    int num_connected(0);
    int i(0);
    while (true) {
        StringSend(pub_socket, "hello");
        if (StringRecv(rep_socket) == "world") {
            ++num_connected;
            StringSend(rep_socket, "");
            if (num_subscriber == num_connected) {
                break;
            }
        } else {
            StringSend(rep_socket, "");
        }
    }
}
}  // namespace myUtils
