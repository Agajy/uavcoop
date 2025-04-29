/*!
 * \file ServerTCP.h
 * \brief server TCP
 * \author theunissen mathilde
 * \date 2022/04/23
 * \version 4.0
 */

#ifndef SERVERTCP_H
#define SERVERTCP_H

#include <IODevice.h>
#include <TcpSocket.h>
#include <Thread.h>
#include <stdint.h>
#include <FrameworkManager.h>
#include <VrpnClient.h>
#include <MetaVrpnObject.h>
#include <Matrix.h>
#include <iostream>
#include <vector>
#include <Vector3D.h>
// #include "receivedData.h"

// #include "Ugv.h"
// #include "TargetBookcase.h"
#include "Streaming.h"

namespace flair {
namespace core {
class FrameworkManager;
class Matrix;
class Socket;
class Message;
class TcpSocket;
class io_data;
}
namespace gui {
class Tab;
class TabWidget;
class DataPlot1D;
}
}

namespace flair {
namespace sensor {

class ServerTCP : public core::Thread, public core::IODevice {

    public :
        ServerTCP(std::string name, uint16_t port, const std::string streaminIp, flair::meta::MetaVrpnObject * vrpnDrone, 
        flair::meta::MetaVrpnObject *targetVrpn, flair::core::Quaternion* currDroneOrientation, uint8_t priority=10);

        ~ServerTCP();

        void getUgvPos(flair::core::Vector3Df &data, float &yaw, float &ugv_acc, float &ugv_steering, bool& valid);

        void resetUgvPos();

        // void sendMessageToUgv(int id, std::string message);

        // int GetUgvIdByName( std::string name);

        // int GetUgvState(int id){return this->Ugv_connected[id].GetState();}

        // int GetUgvTargetId(int id){return this->Ugv_connected[id].GetTarget().GetId();}

        // bool GetUgvReachedTarget(int id){return this->Ugv_connected[id].ReachedTarget();}

        // void SetUgvState(int id, int state){this->Ugv_connected[id].SetState(state);}

        // void SetUgvTarget(int id, Target new_target){this->Ugv_connected[id].SetTarget(new_target);}

        // flair::core::Vector3Df GetUgvPosition(int id){return this->Ugv_connected[id].GetPosition();}

        // flair::core::Vector3Df GetUgvVelocity(int id){return this->Ugv_connected[id].GetVelocity();}

        // void GetUgvCenter(flair::core::Vector2Df &center);

        // void GetUgvCenterVelocity(flair::core::Vector2Df &velocity);

        // int GetNumberOfUGV(){return this->number_of_connections;}

        // float GetUgvCenterYawOrientation();

        // void addRobotsToVrpn();

        // bool EndOfMission();

        // std::vector<Ugv> GetUgvsReady();

        //supervision
        // flair::filter::TargetBookcase* targetManager;

    protected :
    
        flair::core::Matrix *output = nullptr;

        void UpdateFrom(const core::io_data *data){}; // TODO

        void broadcastMessage (std::string message);

    private :

        void Run();

        void sendMessage(std::string clientip,int clientport,std::string mess);

        flair::core::TcpSocket *listeningSocket;
        flair::core::TcpSocket *controlSocket = nullptr;
        int listeningPort;

        int number_of_connections;
        // std::vector<Ugv> Ugv_connected;

        Streaming *stream;
};
}
}

#endif // SERVERTCP_H