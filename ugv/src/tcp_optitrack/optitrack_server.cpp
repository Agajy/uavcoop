#include "optitrack_server.h"
#include <sstream>
#include <iostream>
#include <map>
#include <string>
#include <Quaternion.h>
#include <MetaVrpnObject.h>



using namespace std;
using namespace flair::core;
using namespace flair::meta;
using namespace flair::filter;


PositionServer::PositionServer(const Object* parent, const std::vector<std::string>& names_uav,
    const std::vector<std::string>& names_ugv, const std::string& Ip, const std::string& Port)
    : Thread(parent, "PositionServer", 50, 65536),
      serverSocket(this, "ServerSocketPosition", true, true),
      clientSocket(nullptr),
      usedIp(Ip),
      usedPort(Port),
      isListening(false) {
    SetPeriodMS(50);
    
    for (const auto& name_uav : names_uav) {
        names[name_uav] = new MetaVrpnObject(name_uav);
    }
    for (const auto& name_ugv : names_ugv) {
        names[name_ugv] = new MetaVrpnObject(name_ugv);
    }
}


PositionServer::~PositionServer() {
    isListening = false;
    SafeStop();
    Join();
    if (clientSocket) {
        delete clientSocket;
    }
}

bool PositionServer::initialize() {
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

void PositionServer::transformPosition(ObjectPosition* pose) {
    ObjectPosition CopyPose;
    CopyPose.name = pose->name;
    CopyPose.x = pose->x;
    CopyPose.y = pose->y;
    CopyPose.z = pose->z;
    CopyPose.qx = pose->qx;
    CopyPose.qy = pose->qy;
    CopyPose.qz = pose->qz;
    CopyPose.qw = pose->qw;
    pose->x = CopyPose.y;
    pose->y = -CopyPose.z;
    pose->z = -CopyPose.x;
    pose->qx = CopyPose.qy;
    pose->qy = -CopyPose.qz;
    pose->qz = -CopyPose.qx;
    pose->qw = CopyPose.qw; 
}

void PositionServer::updatePosition() {
    for (std::map<const std::string, MetaVrpnObject*>::iterator it=names.begin(); it!=names.end(); ++it) {
        Vector3Df position;
        Quaternion quaternion;
        names[it->first]->GetPosition(position);
        names[it->first]->GetQuaternion(quaternion);
        ObjectPosition pos{it->first, position.x, position.y, position.z, quaternion.q1, quaternion.q2, quaternion.q3, quaternion.q0};
        transformPosition(&pos);
        positions[it->first]=pos;
    }
}

std::string PositionServer::serializePositions() {
    std::ostringstream oss;
    oss << positions.size() << "\n";
    for (const auto& pair : positions) {
        const ObjectPosition& pos = pair.second;
        oss << pos.name << ";"
            << pos.x << ";" << pos.y << ";" << pos.z << ";"
            << pos.qx << ";" << pos.qy << ";" << pos.qz << ";" << pos.qw << "\n";
    }
    return oss.str();
}

void PositionServer::Run(void) {
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
                updatePosition();
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

