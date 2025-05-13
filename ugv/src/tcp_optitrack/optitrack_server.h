#include <TcpSocket.h>
#include <Thread.h>
#include <string>
#include <vector>
#include <map>


namespace flair {
    namespace meta {
        class MetaVrpnObject;
    }
    namespace core {
        class TcpSocket;
        class Thread;
    }
}


struct ObjectPosition {
    std::string name;
    float x, y, z;
    float qx, qy, qz, qw;
};

class PositionServer : public flair::core::Thread {
public:
    PositionServer(const flair::core::Object* parent, const std::vector<std::string>& names_uav,
    const std::vector<std::string>& names_ugv, const std::string& Ip, const std::string& Port);
    ~PositionServer();
    
    bool initialize();
    void updatePosition();

protected:
    void Run(void) override;

private:
    flair::core::TcpSocket serverSocket;
    flair::core::TcpSocket* clientSocket;
    std::string usedIp;
    std::string usedPort;
    bool isListening;
    std::map<const std::string, flair::meta::MetaVrpnObject*> names;
    std::map<std::string, ObjectPosition> positions;
    std::string serializePositions();
};


