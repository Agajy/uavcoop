#include "CommandServer.h"
#include <iostream>
#include <sstream>
#include <cstring>
#include <cstdlib>

namespace flair {
namespace core {

CommandServer::CommandServer(const Object* parent, const std::string& name, 
                           const std::string& Ip, const std::string& Port)
    : Thread(parent, name, 50, 65536),
      serverSocket(this, "ServerSocketCommand", true, true),
      clientSocket(nullptr),
      usedIp(Ip),
      usedPort(Port),
      command1(0.0f),
      command2(0.0f),
      command3(0.0f),
      isListening(false)
{
    SetPeriodMS(50); // Run every 50ms for better responsiveness
}

CommandServer::~CommandServer() {
    isListening = false;
    SafeStop();
    Join();
    
    if (clientSocket != nullptr) {
        delete clientSocket;
        clientSocket = nullptr;
    }
}

bool CommandServer::initialize() {
    try {
        int portNum = std::stoi(usedPort);
        std::cout << ObjectName() << ": Starting to listen on " << usedIp << ":" << portNum << std::endl;
        
        // Configure and start listening
        serverSocket.Listen(portNum, usedIp);
        isListening = true;
        
        std::cout << ObjectName() << ": Server initialized successfully" << std::endl;
        return true;
    } catch (const std::exception& e) {
        // Err("Failed to initialize server: %s", e.what());
        isListening = false;
        return false;
    }
}

bool CommandServer::parseCommandMessage(const char* message, size_t length) {
    if (!message || length == 0) return false;
    
    std::string msgStr(message, length);
    std::istringstream iss(msgStr);
    std::string token1, token2, token3;
    
    if (std::getline(iss, token1, ';') && 
        std::getline(iss, token2, ';') && 
        std::getline(iss, token3, ';')) {
        try {
            float val1 = std::stof(token1);
            float val2 = std::stof(token2);
            float val3 = std::stof(token3);
            
            command1 = val1;
            command2 = val2;
            command3 = val3;
            
            if (commandCallback) {
                commandCallback(command1, command2, command3);
            }
            return true;
        } catch (const std::exception& e) {
            Warn("Failed to parse command values: %s", e.what());
        }
    }
    return false;
}

bool CommandServer::validateCommands(bool valid){
    in_good_mode = valid;
    return in_good_mode;
}

void CommandServer::Run(void) {
    std::cout << ObjectName() << ": Thread starting..." << std::endl;

    if (!isListening) {
        // Err("Server not initialized. Call initialize() first.");
        return;
    }

    while (!ToBeStopped()) {
        try {
            // Handle new connections
            if (!clientSocket) {
                try {
                    clientSocket = serverSocket.Accept(50000000); // 50ms timeout
                    if (clientSocket) {
                        std::cout << ObjectName() << ": New client connected" << std::endl;
                    }
                } catch (const std::runtime_error&) {
                    // Timeout - normal behavior
                    WaitPeriod();
                    continue;
                }
            }

            // Handle client communication
            if (clientSocket) {
                char buffer[256] = {0};
                try {
                    ssize_t bytesRead = clientSocket->RecvMessage(buffer, sizeof(buffer) - 1, 25000000);
                    
                    if (bytesRead > 0) {
                        buffer[bytesRead] = '\0';
                        std::cout << ObjectName() << ": Received: " << buffer << std::endl;
                        
                        if (parseCommandMessage(buffer, bytesRead)) {
                            // Vérifier si les commandes sont dans des limites acceptables
                            if (in_good_mode) {
                                clientSocket->SendMessage("True", 4, 25000000);
                            } else {
                                clientSocket->SendMessage("False", 5, 25000000);
                            }
                        } else {
                            clientSocket->SendMessage("False", 5, 25000000);
                        }
                    } else if (bytesRead == 0) {
                        std::cout << ObjectName() << ": Client disconnected" << std::endl;
                        delete clientSocket;
                        clientSocket = nullptr;
                    }
                } catch (const std::exception& e) {
                    Err("Communication error: %s", e.what());
                    delete clientSocket;
                    clientSocket = nullptr;
                }
            }
        } catch (const std::exception& e) {
            Err("Unexpected error: %s", e.what());
        }
        
        WaitPeriod();
    }

    if (clientSocket) {
        delete clientSocket;
        clientSocket = nullptr;
    }

    std::cout << ObjectName() << ": Thread stopped." << std::endl;
}

void CommandServer::setCommandCallback(std::function<void(float, float, float)> callback) {
    commandCallback = callback;
}

float CommandServer::getCommand1() const { return command1; }
float CommandServer::getCommand2() const { return command2; }
float CommandServer::getCommand3() const { return command3; }

} // namespace core
} // namespace flair