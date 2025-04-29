#include "ServerTCP.h"

#include <cstring>
#include <string>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <stdexcept>

using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;

#define UGV_READY           1
#define UGV_BUSY            2
#define UGV_END  	 		3

namespace flair {
namespace sensor {

ServerTCP::ServerTCP(std::string name, uint16_t port, const std::string streaminIp, 
					flair::meta::MetaVrpnObject * vrpnDrone, flair::meta::MetaVrpnObject *targetVrpn,
					flair::core::Quaternion* currDroneOrientation, 
					uint8_t priority) : 
					Thread(getFrameworkManager(),name,priority), IODevice(getFrameworkManager(),name){
	
	const bool blocking = true;
	this->listeningSocket = new TcpSocket(getFrameworkManager(), "listening_socket", blocking, blocking);
    this->listeningPort = port;
    // this->number_of_connections = number_of_ugv;
    // this->Ugv_connected = liste;

        //IMAGE STREAMING
    #ifdef STREAMING
        this->stream = new ::Streaming(streaminIp, vrpnDrone, targetVrpn, currDroneOrientation);
        getFrameworkManager()->AddDeviceToLog(stream);
    #ifdef SIMU
        this->stream->Start();
    #endif
    #endif

    //SUPERVISION
    // this->targetManager = new TargetBookcase(this->stream, "targets_position_manager", vrpnDrone);

    // this->broadcastMessage("S 1");
}

// get received data fromground station
void ServerTCP::getUgvPos(Vector3Df &data, float &yaw, float &ugv_acc, float &ugv_steering, bool& valid){
	data.x = this->stream->last_receivedData.x;
	data.y = this->stream->last_receivedData.y;
	data.z = this->stream->last_receivedData.z;
	yaw = this->stream->last_receivedData.yaw;
	ugv_acc = this->stream->last_receivedData.ugv_acc;
	ugv_steering = this->stream->last_receivedData.ugv_steering;
	valid = this->stream->last_receivedData.valid;
}

void ServerTCP::resetUgvPos(){
	this->stream->last_receivedData.x = 0;
	this->stream->last_receivedData.y = 0;
	this->stream->last_receivedData.z = 0;
	this->stream->last_receivedData.yaw = 0;
	this->stream->last_receivedData.ugv_acc = 0;
	this->stream->last_receivedData.ugv_steering = 0;
	this->stream->last_receivedData.valid = false;
}

ServerTCP::~ServerTCP(){
	SafeStop();
 	Join();
  	delete this->listeningSocket;
}

// int ServerTCP::GetUgvIdByName(std::string name){
// 	for (int i=0; i<this->number_of_connections; i++){
// 		if(name == this->Ugv_connected[i].GetName()){
// 			return i;
// 		}
// 	}
// 	return -1;
// }

// void ServerTCP::GetUgvCenter(flair::core::Vector2Df &center){
// 	Vector3Df temp (0,0,0);
//     for (int i=0; i<this->number_of_connections; i++){
//         temp += GetUgvPosition(i);
//     }
//     center.x = temp.x/this->number_of_connections;
//     center.y = temp.y/this->number_of_connections;
// }

// void ServerTCP::GetUgvCenterVelocity(flair::core::Vector2Df &velocity){
// 	Vector3Df temp;
//     for (int i=0; i<this->number_of_connections; i++){
//         temp += GetUgvVelocity(i);
//     }
//     velocity.x = temp.x/this->number_of_connections;
//     velocity.y = temp.y/this->number_of_connections;
// }

// float ServerTCP::GetUgvCenterYawOrientation(){
// 	flair::core::Vector2Df vi;
// 	this->GetUgvCenterVelocity(vi);
// 	float yaw = atan2(vi.y, vi.x);
// 	return yaw;
// }


// void ServerTCP::addRobotsToVrpn(){ //ConnectionType_t type
// 	for (int i=0; i<this->number_of_connections; i++){
//         this->Ugv_connected[i].AddVrpnObject();
//     }
// }

// bool ServerTCP::EndOfMission(){
// 	for (int i=0; i<number_of_connections; i++){
// 		if (this->Ugv_connected[i].GetState() != UGV_END){
// 			return false;
// 		}
// 	}
// 	return true;
// }

// std::vector<Ugv> ServerTCP::GetUgvsReady(){
// 	std::vector<Ugv> listeRobots;
// 	for (int i=0; i<number_of_connections; i++){
// 		if (this->Ugv_connected[i].GetState() == UGV_READY){
// 			listeRobots.push_back(this->Ugv_connected[i]);
// 		}
// 	}
// 	return listeRobots;
// }


// void ServerTCP::sendMessageToUgv(int id, std::string message){
// 	std::cout << "message vers " << Ugv_connected[id].GetName() << ", texte = " << message << std::endl; 
// 	this->sendMessage(Ugv_connected[id].GetIpAdress(),Ugv_connected[id].GetPort(),message);
// }

void ServerTCP::sendMessage(std::string clientip,int clientport,std::string mess)
{
	const bool blocking = true;
	TcpSocket* s = new TcpSocket((Thread *)this, "socket", blocking, !blocking);
	bool connected = false;
	while (!connected) {
		try {
	    	s->Connect(clientport, clientip, 100000000);
	    	connected = true;
	    } 
	    catch (std::runtime_error e) {
	    	std::cout << e.what() << std::endl;
	      // timeout
	      if (ToBeStopped())
	        return ;
	    }
	}
	bool sizeSent = false;
	while (!sizeSent) {
		try {
	    	s->WriteUInt32(mess.size(), 0);
	    	sizeSent = true;
	    } 
	    catch (std::runtime_error e) {
	      // timeout
	      if (ToBeStopped())
	        return ;
	    }
	}
	bool MessageSent = false;
	while (!MessageSent) {
		try {
	    	s->SendMessage(mess.c_str(), mess.size(), 0);
	    	MessageSent = true;
	    } 
	    catch (std::runtime_error e) {
	      // timeout
	      if (ToBeStopped())
	        return ;
	    }
	}
	Thread::Info("Debug: sent message to server \n");
	
    delete s;
}

// void ServerTCP::broadcastMessage (std::string message){
// 	for (int i = 0; i<this->number_of_connections; i++){
// 		this->sendMessage(Ugv_connected[i].GetIpAdress(),Ugv_connected[i].GetPort(),message);
// 	}
// }

void ServerTCP::Run(){
	// Server initialisation
	this->listeningSocket->Listen(this->listeningPort);
	int current_connection_nb = 0;

	// Thread initialisation
	SetPeriodMS(20); //50Hz

	// boucle infinie
	while(!ToBeStopped()){
		//WaitPeriod();
		while (!controlSocket && !ToBeStopped()) {
			//Printf("try \n");
			try {
		    	controlSocket = listeningSocket->Accept(100000000);
		    } 
		    catch (std::logic_error &e) {
		      Thread::Err("%s\n",e.what());
		      if (ToBeStopped())
		        return ;
		    } 
		    catch (std::runtime_error e) {
		      // timeout
		      if (ToBeStopped())
		        return ;
		    }
		}
  		Thread::Info("Debug: accept connection \n");

  			// lecture de la taille du message
		int stringSize;
		bool stringSizeRead = false;
		while (!stringSizeRead) {
	     	try {
		      	//Thread::Info("Debug: wait for reading\n");
		        stringSize = controlSocket->ReadUInt32(100000000);
		        Thread::Info("Debug: read %d\n", stringSize);
		        stringSizeRead = true;
		    } 
	      	catch (std::runtime_error e) {
		      	//std::cout << e.what() << std::endl;
		        // timeout
		        if (ToBeStopped()){
		        	Thread::Info("Debug: STOP Thread %d\n");
		          	return ;
		      	}
	      	}
	   	}
			// lecture du message
		bool messageRead = false;
	    std::string message_entrant;
	    while (!messageRead) {
			try {
				message_entrant = controlSocket->ReadString(stringSize, 100000000);
				
				Thread::Info("\nDebug: read message\n\n");
				// std::cout << message_entrant<<std::endl<<std::endl;

				messageRead = true;
			} 
			catch (std::runtime_error e) {
				// timeout
				if (ToBeStopped())
					return ;
				}
	    }
  		
	    char com;
		char name[20];
		Vector3Df target;
  		
		// manages ugv
		/*if (message_entrant[0] == 'R'){
			
    		sscanf(message_entrant.c_str(),"%c %s %f %f %f", &com, name, &(target.x), &(target.y), &(target.z));	            
        	
    		// ugv can go to a other mission
    		std::string name_string(name);
    		int ugv_id = this->GetUgvIdByName(name_string);
    		if (ugv_id == -1){
    			Thread::Err("ugv named %s can not be find in the list", name);
    		}
    		else{
    			this->SetUgvState(ugv_id, UGV_READY);
    		}

    		// target : state visited 
    		this->targetManager->SetTargetVisited(ugv_id);
    	}*/
  		
	    	// fin de la connection
	    delete controlSocket;
	    controlSocket = nullptr;
	   	
	}
}

} // end namespace sensor
} // end namespace flair