#ifndef COMMAND_SERVER_H
#define COMMAND_SERVER_H

#include <TcpSocket.h>
#include <Thread.h>
#include <string>
#include <functional>

namespace flair {
namespace core {

/**
 * Server class to handle command reception over TCP/IP
 */
class CommandServer : public Thread {
public:
    /**
     * Constructor
     * @param parent Parent object
     * @param name Object name
     * @param Ip IP address to listen on
     * @param Port Port to listen on
     */
    CommandServer(const Object* parent, const std::string& name, 
                 const std::string& Ip, const std::string& Port);
    
    /**
     * Destructor - ensures proper cleanup of resources
     */
    ~CommandServer();
    
    /**
     * Initialize the server and start listening
     * @return true if successful, false otherwise
     */
    bool initialize();
    
    /**
     * Set callback for received command values
     * @param callback Function to call when command is received (val1, val2, val3)
     */
    void setCommandCallback(std::function<void(float, float, float, float)> callback);
    
    /**
     * Get the first received command value
     * @return First command value
     */
    float getCommand1() const;
    
    /**
     * Get the second received command value
     * @return Second command value
     */
    float getCommand2() const;

    /**
     * Get the third received command value
     * @return Third command value
     */
    float getCommand3() const;


    /**
     * Get the fourth received command value
     * @return fourth command value
     */
    float getCommand4() const;

    /**
     * Check if server is currently listening
     * @return true if server is listening for connections
     */
    bool isServerListening() const { return isListening; }

    /**
     * Check if a client is currently connected
     * @return true if a client is connected
     */
    bool hasConnectedClient() const { return clientSocket != nullptr; }

    bool validateCommands(bool valid);
    /**
     * Get the command values
     * @return true if command values are valid
     */

protected:
    /**
     * Thread run method - handles connections and message processing
     */
    void Run(void) override;

private:
    /**
     * Parse received message and extract command values
     * @param message Received message buffer
     * @param length Length of the message
     * @return true if parsing was successful
     */
    bool parseCommandMessage(const char* message, size_t length);

    // Network related members
    TcpSocket serverSocket;        ///< Server socket for listening
    TcpSocket* clientSocket;       ///< Current client connection
    std::string usedIp;           ///< IP address to listen on
    std::string usedPort;         ///< Port to listen on
    bool isListening;             ///< Server listening status

    // Command related members
    float command1;               ///< First command value
    float command2;               ///< Second command value
    float command3;               ///< Third command value
    float command4;               ///< Fourth command value
    std::function<void(float, float, float, float)> commandCallback;  ///< Callback for received commands
    

    bool in_good_mode=false;    ///< Flag to indicate if commands are valid
    // Constants
    static constexpr size_t MAX_BUFFER_SIZE = 256;     ///< Maximum message buffer size
    static constexpr int ACCEPT_TIMEOUT_MS = 50;       ///< Accept timeout in milliseconds
    static constexpr int RECEIVE_TIMEOUT_MS = 25;      ///< Receive timeout in milliseconds
};

} // end namespace core
} // end namespace flair

#endif // COMMAND_SERVER_H