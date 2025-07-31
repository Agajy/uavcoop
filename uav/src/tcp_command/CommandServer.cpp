#include "CommandServer.h"
#include <iostream>
#include <sstream>
#include <cstring>
#include <cstdlib>
#include <chrono>

namespace flair {
namespace core {

CommandServer::CommandServer(const Object* parent, const std::string& name, 
                           const std::string& Ip, const std::string& Port)
    : Thread(parent, name, 10, 65536), // Reduced period to 10ms for better responsiveness
      serverSocket(this, "ServerSocketCommand", true, true),
      clientSocket(nullptr),
      usedIp(Ip),
      usedPort(Port),
      command1(0.0f),
      command2(0.0f),
      command3(0.0f),
      command4(0.0f),
      command5(0.0f),
      isListening(false)
{
    // Don't call SetPeriodMS here, it's already set in constructor
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
        Err("Failed to initialize server: %s", e.what());
        isListening = false;
        return false;
    }
}

bool CommandServer::parseCommandMessage(const char* message, size_t length) {
    if (!message || length == 0) return false;
    
    std::string msgStr(message, length);
    std::istringstream iss(msgStr);
    std::string token1, token2, token3, token4, token5;
    
    if (std::getline(iss, token1, ';') && 
        std::getline(iss, token2, ';') && 
        std::getline(iss, token3, ';') &&
        std::getline(iss, token4, ';') &&
        std::getline(iss, token5, ';')) {
        try {
            float val1 = std::stof(token1);
            float val2 = std::stof(token2);
            float val3 = std::stof(token3);
            float val4 = std::stof(token4);
            float val5 = std::stof(token5);
            
            command1 = val1;
            command2 = val2;
            command3 = val3;
            command4 = val4;
            command5 = val5;
            
            if (commandCallback) {
                commandCallback(command1, command2, command3, command4, command5);
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

    // Optionally increase period to 20ms for more margin
    SetPeriodMS(20);

    if (!isListening) {
        Err("Server not initialized. Call initialize() first.");
        return;
    }

    // Use a counter to reduce connection check frequency
    int connectionCheckCounter = 0;
    const int CONNECTION_CHECK_PERIOD = 5; // Check for new connections every 5 cycles

    while (!ToBeStopped()) {
        auto loop_start = std::chrono::steady_clock::now();
        try {
            // Handle new connections less frequently
            if (!clientSocket && (connectionCheckCounter % CONNECTION_CHECK_PERIOD == 0)) {
                try {
                    // Shorter timeout for Accept to avoid blocking
                    clientSocket = serverSocket.Accept(5000000); // 5ms timeout
                    if (clientSocket) {
                        std::cout << ObjectName() << ": New client connected" << std::endl;
                    }
                } catch (const std::runtime_error&) {
                    // Timeout - normal behavior, continue
                }
            }
            connectionCheckCounter++;

            // Handle client communication
            if (clientSocket) {
                char buffer[256] = {0};
                try {
                    // Very short timeout for receive to avoid blocking
                    ssize_t bytesRead = clientSocket->RecvMessage(buffer, sizeof(buffer) - 1, 2000000); // 2ms timeout
                    
                    if (bytesRead > 0) {
                        buffer[bytesRead] = '\0';
                        std::cout << ObjectName() << ": Received: " << buffer << std::endl;
                        
                        if (parseCommandMessage(buffer, bytesRead)) {
                            if (in_good_mode) {
                                clientSocket->SendMessage("True", 4, 2000000); // 2ms timeout
                            } else {
                                clientSocket->SendMessage("False", 5, 2000000);
                            }
                        } else {
                            clientSocket->SendMessage("False", 5, 2000000);
                        }
                    } else if (bytesRead == 0) {
                        std::cout << ObjectName() << ": Client disconnected" << std::endl;
                        delete clientSocket;
                        clientSocket = nullptr;
                    }
                } catch (const std::exception& e) {
                    // Don't log timeout errors, they're normal
                    if (std::string(e.what()).find("timeout") == std::string::npos) {
                        Err("Communication error: %s", e.what());
                        delete clientSocket;
                        clientSocket = nullptr;
                    }
                }
            }
        } catch (const std::exception& e) {
            Err("Unexpected error: %s", e.what());
        }
        
        // This is crucial - use WaitPeriod() to maintain timing
        WaitPeriod();

        // Debug: print loop duration
        auto loop_end = std::chrono::steady_clock::now();
        auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end - loop_start).count();
        if (duration_ms > 20) {
            std::cout << ObjectName() << ": Warning - loop took " << duration_ms << " ms" << std::endl;
        }
    }

    if (clientSocket) {
        delete clientSocket;
        clientSocket = nullptr;
    }

    std::cout << ObjectName() << ": Thread stopped." << std::endl;
}

void CommandServer::setCommandCallback(std::function<void(float, float, float, float, float)> callback) {
    commandCallback = callback;
}

float CommandServer::getCommand1() const { return command1; }
float CommandServer::getCommand2() const { return command2; }
float CommandServer::getCommand3() const { return command3; }
float CommandServer::getCommand4() const { return command4; }
float CommandServer::getCommand5() const { return command5; }

} // namespace core
} // namespace flair