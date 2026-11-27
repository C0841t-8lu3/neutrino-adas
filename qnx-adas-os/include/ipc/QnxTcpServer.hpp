#pragma once

#include "adas/QnxAdasController.hpp"
#include <atomic>

namespace ipc {

class QnxTcpServer {
public:
    explicit QnxTcpServer(int port = 9090);
    ~QnxTcpServer();

    // Prevent copies
    QnxTcpServer(const QnxTcpServer&) = delete;
    QnxTcpServer& operator=(const QnxTcpServer&) = delete;

    // Server lifecycle
    bool start();
    void run();
    void stop();

    int getPort() const { return port_; }
    bool isRunning() const { return running_; }

private:
    int port_;
    int server_fd_ = -1;
    std::atomic<bool> running_{false};
    adas::QnxAdasController controller_;

    void handleClient(int client_sock);
};

} // namespace ipc
