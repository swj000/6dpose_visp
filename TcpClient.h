#pragma once
#include <string>
#include <winsock2.h>

class TcpClient
{
public:
    TcpClient(const std::string& ip, int port);
    ~TcpClient();

    bool connectToServer();
    bool sendCommand(const std::string& command);
    std::string receiveResponse();

private:
    std::string ip_;
    int port_;
    SOCKET sock_;
    bool initialized_;

    bool initializeWinsock();
    void cleanup();
};