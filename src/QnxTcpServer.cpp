#include "ipc/QnxTcpServer.hpp"
#include "ipc/TelemetryCodec.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <vector>
#include <pthread.h>
#include <sched.h>

namespace ipc {

QnxTcpServer::QnxTcpServer(int port)
    : port_(port), server_fd_(-1), running_(false), controller_() {}

QnxTcpServer::~QnxTcpServer() {
    stop();
}

void QnxTcpServer::stop() {
    running_ = false;
    if (server_fd_ >= 0) {
        close(server_fd_);
        server_fd_ = -1;
    }
}

bool QnxTcpServer::start() {
    std::cout << "[QNX RTOS] Initializing High-Speed Real-Time Socket Server on Port " << port_ << "...\n";

    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
        perror("socket failed");
        return false;
    }

    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in address;
    std::memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port_);

    if (bind(server_fd_, (struct sockaddr*)&address, sizeof(address)) < 0) {
        perror("bind failed");
        close(server_fd_);
        server_fd_ = -1;
        return false;
    }

    if (listen(server_fd_, 3) < 0) {
        perror("listen failed");
        close(server_fd_);
        server_fd_ = -1;
        return false;
    }

    std::cout << "[QNX RTOS] ADAS L3 Safety Kernel Listening for 3D Vehicle Link...\n";
    running_ = true;
    return true;
}

void QnxTcpServer::handleClient(int client_sock) {
    // 1. Enforce TCP_NODELAY on accepted client socket
    int nodelay_opt = 1;
    setsockopt(client_sock, IPPROTO_TCP, TCP_NODELAY, &nodelay_opt, sizeof(nodelay_opt));

    // 2. Real-Time thread priority under QNX Neutrino RTOS
#ifdef __QNXNTO__
    struct sched_param param;
    param.sched_priority = 20; // High real-time priority in QNX range
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) == 0) {
        std::cout << "[QNX RTOS] Elevated client thread to SCHED_FIFO priority 20 (Real-Time)\n";
    } else {
        std::cout << "[QNX RTOS] Note: Running at standard priority (SCHED_FIFO elevation skipped/unprivileged)\n";
    }
#endif

    char buffer[8192];
    std::string line_buffer;
    line_buffer.reserve(8192);

    // Reset controller state for new client connection
    controller_.reset();

    // Pre-allocated reusable obstacle buffer and ego state (zero dynamic heap reallocation in hot loop)
    std::vector<adas::Obstacle> obstacles;
    obstacles.reserve(32);
    adas::VehicleState ego;

    while (running_) {
        ssize_t bytes_read = recv(client_sock, buffer, sizeof(buffer) - 1, 0);
        if (bytes_read <= 0) {
            std::cout << "[QNX RTOS] 3D Car Disconnected.\n";
            break;
        }

        buffer[bytes_read] = '\0';
        line_buffer.append(buffer);

        size_t newline_pos = 0;
        while ((newline_pos = line_buffer.find('\n')) != std::string::npos) {
            std::string packet = line_buffer.substr(0, newline_pos);
            line_buffer.erase(0, newline_pos + 1);

            float dt = 0.016f;

            if (TelemetryCodec::parseTelemetryJson(packet, ego, obstacles, dt)) {
                adas::AdasOutput out = controller_.step(ego, obstacles, dt);
                std::string resp = TelemetryCodec::generateResponseJson(out);
                send(client_sock, resp.c_str(), resp.length(), 0);
            }
        }
    }

    close(client_sock);
}

void QnxTcpServer::run() {
    if (server_fd_ < 0) {
        if (!start()) {
            return;
        }
    }

    while (running_) {
        sockaddr_in client_addr;
        socklen_t addrlen = sizeof(client_addr);
        std::cout << "[QNX RTOS] Waiting for 3D Car Connection...\n";

        int client_sock = accept(server_fd_, (struct sockaddr*)&client_addr, &addrlen);
        if (client_sock < 0) {
            if (!running_) break;
            perror("accept failed");
            continue;
        }

        char client_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
        std::cout << "[QNX RTOS] 3D Car Connected from: " << client_ip << "!\n";

        handleClient(client_sock);
    }
}

} // namespace ipc
