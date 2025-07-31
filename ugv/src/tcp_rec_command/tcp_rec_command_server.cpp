#include "tcp_rec_command_server.h"
#include <sstream>
#include <iostream>
#include <map>
#include <string>
#include <MetaVrpnObject.h>




using namespace std;
using namespace flair::core;
using namespace flair::meta;
using namespace flair::filter;


RecCommandServer::RecCommandServer(const Object* parent,std::string name_object, const std::string& Ip, const std::string& Port)
    : Thread(parent, "RecCommandServer", 50, 65536),
      serverSocket(this, "ServerSocketPosition", true, true),
      clientSocket(nullptr),
      usedIp(Ip),
      usedPort(Port),
      isListening(false) {
    SetPeriodMS(50);
    
    position.name = name_object;
    position.x = 0.0f;
    position.y = 0.0f;
    position.z = 0.0f;
    position.qx = 0.0f;
    position.qy = 0.0f;
    position.qz = 0.0f;
    position.qw = 1.0f; 

}

RecCommandServer::~RecCommandServer() {
    isListening = false;
    SafeStop();
    Join();
    if (clientSocket) {
        delete clientSocket;
    }
}

bool RecCommandServer::initialize() {
    try {
        int portNum = std::stoi(usedPort);
        serverSocket.Listen(portNum, usedIp);
        isListening = true;
        return true;
    } catch (const std::exception& e) {
        isListening = false;
        return false;
    }
}


void RecCommandServer::updatePosition(flair::core::Vector3Df pose, flair::core::Quaternion quaternion) {
    position.x = pose.x;
    position.y = pose.y;
    position.z = pose.z;
    position.qx = quaternion.q0;
    position.qy = quaternion.q1;
    position.qz = quaternion.q2;
    position.qw = quaternion.q3;
}

std::string RecCommandServer::serializePositions() {
    std::ostringstream oss;
    oss << 1 << "\n";
    oss << position.name << ";"
    << position.x << ";" << position.y << ";" << position.z << ";"
    << position.qx << ";" << position.qy << ";" << position.qz << ";" << position.qw << "\n";
    return oss.str();
}

void RecCommandServer::Run(void) {
    if (!isListening) return;

    while (!ToBeStopped()) {
        try {
            if (!clientSocket) {
                try {
                    clientSocket = serverSocket.Accept(50000000);
                } catch (const std::runtime_error&) {
                    WaitPeriod();
                    continue;
                }
            }
            
            if (clientSocket) {
                std::string data = serializePositions();
                try {
                    clientSocket->SendMessage(data.c_str(), data.length(), 25000000);
                } catch (const std::exception& e) {
                    delete clientSocket;
                    clientSocket = nullptr;
                }
            }
        } catch (const std::exception& e) {
            if (clientSocket) {
                delete clientSocket;
                clientSocket = nullptr;
            }
        }
        WaitPeriod();
    }
}

