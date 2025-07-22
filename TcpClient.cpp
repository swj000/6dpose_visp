#include "TcpClient.h"
#include <ws2tcpip.h>
#include <iostream>

#pragma comment(lib, "Ws2_32.lib")

TcpClient::TcpClient(const std::string& ip, int port)
    : ip_(ip), port_(port), sock_(INVALID_SOCKET), initialized_(false)
{
    initialized_ = initializeWinsock();
}

TcpClient::~TcpClient()
{
    if (sock_ != INVALID_SOCKET)
        closesocket(sock_);
    cleanup();
}

bool TcpClient::initializeWinsock()
{
    WSADATA wsaData;
    return WSAStartup(MAKEWORD(2, 2), &wsaData) == 0;
}

void TcpClient::cleanup()
{
    WSACleanup();
}

bool TcpClient::connectToServer()
{
    if (!initialized_)
        return false;

    sock_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock_ == INVALID_SOCKET)
        return false;

    sockaddr_in serverAddr = {};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port_);
    inet_pton(AF_INET, ip_.c_str(), &serverAddr.sin_addr);

    return connect(sock_, (SOCKADDR*)&serverAddr, sizeof(serverAddr)) != SOCKET_ERROR;
}

bool TcpClient::sendCommand(const std::string& command)
{
    return send(sock_, command.c_str(), command.length(), 0) != SOCKET_ERROR;
}

std::string TcpClient::receiveResponse()
{
    char buffer[1024] = {};
    int bytesReceived = recv(sock_, buffer, sizeof(buffer) - 1, 0);
    if (bytesReceived > 0)
    {
        buffer[bytesReceived] = '\0';
        return std::string(buffer);
    }
    return "";
}
