#include "ipc/QnxTcpServer.hpp"
#include "sim/ConsoleSelfTest.hpp"
#include <string>
#include <cstdlib>

int main(int argc, char* argv[]) {
    int port = 9090;
    bool server_mode = false;

    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--server" || std::string(argv[i]) == "-s") {
            server_mode = true;
        } else if (std::string(argv[i]) == "--port" && i + 1 < argc) {
            port = std::atoi(argv[++i]);
            server_mode = true;
        }
    }

    if (server_mode) {
        ipc::QnxTcpServer server(port);
        server.run();
    } else {
        sim::ConsoleSelfTest::run(120);
    }

    return 0;
}
