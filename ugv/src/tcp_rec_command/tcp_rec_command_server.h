#include <TcpSocket.h>
#include <Thread.h>
#include <string>
#include <vector>
#include <map>
#include <Vector3D.h>
#include <Quaternion.h>


namespace flair {
    namespace meta {
        class MetaVrpnObject;
    }
    namespace core {
        class TcpSocket;
        class Thread;
        typedef Vector3D<float> Vector3Df;
    }
}

struct ObjectPosition {
    std::string name;
    float x, y, z;
    float qx, qy, qz, qw;
};

class RecCommandServer : public flair::core::Thread {
public:
    RecCommandServer(const Object* parent,std::string name_object, const std::string& Ip, const std::string& Port);
    ~RecCommandServer();
    

    bool initialize();
    void updatePosition(flair::core::Vector3Df pose, flair::core::Quaternion quaternion);

protected:
    void Run(void) override;

private:
    flair::core::TcpSocket serverSocket;
    flair::core::TcpSocket* clientSocket;
    std::string usedIp;
    std::string usedPort;
    bool isListening;
    ObjectPosition position;
    std::string serializePositions();

};


