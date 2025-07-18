//  created:    2025/03/01
//  filename:   test_fleet.h
//
//  author:     Aurelien Garreau
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    
//
//
/*********************************************************************/

#include "test_fleet.h"
#include "read_csv.h"
#include "tcp_command/CommandServer.h"
// #include "tcp_rec_command/tcp_rec_command_server.h"
#include<unistd.h>
#include <TargetController.h>
#include <Uav.h>
#include <GroupBox.h>
#include <GridLayout.h>
#include <PushButton.h>
#include <DoubleSpinBox.h>
#include <DataPlot1D.h>
#include <DataPlot2D.h>
#include <FrameworkManager.h>
#include <VrpnClient.h>
#include <MetaVrpnObject.h>
#include <TrajectoryGenerator2DCircle.h>
#include <Matrix.h>
#include <cmath>
#include <Tab.h>
#include <Pid.h>
#include <PidThrust.h>
#include <Ahrs.h>
#include <AhrsData.h>
// #include <thread>
#include <Thread.h>
#include <cmath> 
#define PI ((float)3.14159265358979323846)


using namespace std;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::sensor;
using namespace flair::filter;
using namespace flair::meta;


//,uint16_t listeningPort): UavStateMachine(controller), behaviourMode(BehaviourMode_t::Default), vrpnLost(false) {

test_fleet::test_fleet(TargetController *controller, const std::string& name, const std::string& Ip, const std::string& Port): UavStateMachine(controller), behaviourMode(BehaviourMode_t::Default), vrpnLost(false), running(true) {
    Uav* uav=GetUav();

    VrpnClient* vrpnclient=new VrpnClient("vrpn", uav->GetDefaultVrpnAddress(),80,uav->GetDefaultVrpnConnectionType());
    
    if(vrpnclient->ConnectionType()==VrpnClient::Xbee) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName(),(uint8_t)0);
    } else if (vrpnclient->ConnectionType()==VrpnClient::Vrpn) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName());
    } else if (vrpnclient->ConnectionType()==VrpnClient::VrpnLite) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName());
    }
    
    //set vrpn as failsafe altitude sensor for mamboedu as us in not working well for the moment
    if(uav->GetType()=="mamboedu") {
      SetFailSafeAltitudeSensor(uavVrpn->GetAltitudeSensor());
    }
    
    getFrameworkManager()->AddDeviceToLog(uavVrpn);
    vrpnclient->Start();
    
    uav->GetAhrs()->YawPlot()->AddCurve(uavVrpn->State()->Element(2),DataPlot::Green);
																 
    startCircle=new PushButton(GetButtonsLayout()->NewRow(),"Start_circle");
    stopCircle=new PushButton(GetButtonsLayout()->LastRowLastCol(),"stop_circle");
    // startFollowUGV=new PushButton(GetButtonsLayout()->NewRow(),"follow_ugv");
    // stopFollowUGV=new PushButton(GetButtonsLayout()->LastRowLastCol(),"stop_follow_ugv");
    positionHold=new PushButton(GetButtonsLayout()->LastRowLastCol(),"position hold");
    followPathUAV=new PushButton(GetButtonsLayout()->NewRow(),"follow path");
    UAVControlTCP=new PushButton(GetButtonsLayout()->LastRowLastCol(),"UAV control by TCP");

    pathUAV = readcsv();
    path_length=pathUAV.size();

    circle=new TrajectoryGenerator2DCircle(vrpnclient->GetLayout()->NewRow(),"circle");
    uavVrpn->xPlot()->AddCurve(circle->GetMatrix()->Element(0,0),DataPlot::Blue);
    uavVrpn->yPlot()->AddCurve(circle->GetMatrix()->Element(0,1),DataPlot::Blue);
    uavVrpn->VxPlot()->AddCurve(circle->GetMatrix()->Element(1,0),DataPlot::Blue);
    uavVrpn->VyPlot()->AddCurve(circle->GetMatrix()->Element(1,1),DataPlot::Blue);
    uavVrpn->XyPlot()->AddCurve(circle->GetMatrix()->Element(0,1),circle->GetMatrix()->Element(0,0),DataPlot::Blue,"circle");

    MatrixDescriptor *desc = new MatrixDescriptor(2, 1);
    desc->SetElementName(0, 0, "xpath");
    desc->SetElementName(1, 0, "ypath");
    matrix_path = new Matrix(this, desc, floatType, "matrix_path");
    delete desc;
    matrix_path->SetValueNoMutex(0, 0, 0.0);
    matrix_path->SetValueNoMutex(1, 0, 0.0);
    uavVrpn->XyPlot()->AddCurve(matrix_path->Element(1,0),matrix_path->Element(0,0),DataPlot::Green,"path follower");



    uX=new Pid(setupLawTab->At(1,0),"u_x");
    uX->UseDefaultPlot(graphLawTab->NewRow());
    uY=new Pid(setupLawTab->At(1,1),"u_y");
    uZ_custom=new PidThrust(setupLawTab->At(0,1),"u_z custom");
    uY->UseDefaultPlot(graphLawTab->LastRowLastCol());

    customReferenceOrientation= new AhrsData(this,"reference");
    uav->GetAhrs()->AddPlot(customReferenceOrientation,DataPlot::Yellow);
    AddDataToControlLawLog(customReferenceOrientation);
    AddDeviceToControlLawLog(uX);
    AddDeviceToControlLawLog(uY);

    customOrientation=new AhrsData(this,"orientation");

    v = new DoubleSpinBox(GetButtonsLayout()->NewRow(), "step*s", " step*s", 40, 100, 1.0,0.1,80);


    // Création du serveur reception command
    commandServerUAV = new CommandServer(nullptr,name,Ip,Port);

    // Initialiser le serveur avant de démarrer le thread
    if (commandServerUAV->initialize()) {
        // Configuration du callback si nécessaire
        commandServerUAV->setCommandCallback([](float v1, float v2, float v3, float v4) {
            std::cout << "Received commands: " << v1 << ", " << v2 << ", " << v3 << ", " << v4 << std::endl;
        });
    
        // Démarrer le thread seulement si l'initialisation a réussi
        commandServerUAV->Start();
    } else {
        std::cerr << "Failed to initialize command server" << std::endl;
    }

    commandRecorder = new RecCommandServer(nullptr,name, "127.0.0.1","62733");

    if (commandRecorder->initialize()) {
        commandRecorder->Start();  // Lancement du thread

    } else {
        std::cerr << "Failed to initialize command recorder" << std::endl;
    }
   

}

test_fleet::~test_fleet() {
    running = false;  // Signale au thread de s'arrêter
    commandServerUAV->Join();
    delete commandServerUAV;
    commandRecorder->Join();
    delete commandRecorder;
}


bool test_fleet::Get_flag_followpath(void){
    return flag_followpath;
}

const AhrsData *test_fleet::GetOrientation(void) const {
    Quaternion uavQuaternion(1,0,0,0);
    
    if (!line_detected) {
        //get yaw from vrpn
        uavVrpn->GetQuaternion(uavQuaternion);
    }
    //get roll, pitch and w from imu
    Quaternion ahrsQuaternion;
    Vector3Df ahrsAngularSpeed;
    GetDefaultOrientation()->GetQuaternionAndAngularRates(ahrsQuaternion, ahrsAngularSpeed);

    Euler ahrsEuler=ahrsQuaternion.ToEuler();
    ahrsEuler.yaw=uavQuaternion.ToEuler().yaw;
    Quaternion mixQuaternion=ahrsEuler.ToQuaternion();

    customOrientation->SetQuaternionAndAngularRates(mixQuaternion,ahrsAngularSpeed);

    return customOrientation;
}

void test_fleet::AltitudeValues(float &z,float &dz) const{
    Vector3Df uav_pos,uav_vel;

    uavVrpn->GetPosition(uav_pos);
    uavVrpn->GetSpeed(uav_vel);
    //z and dz must be in uav's frame
    z=-uav_pos.z;
    dz=-uav_vel.z;
}

AhrsData *test_fleet::GetReferenceOrientation(void) {
    Vector2Df pos_err, vel_err; // in Uav coordinate system
    float yaw_ref;
    Euler refAngles;

    PositionValues(pos_err, vel_err, yaw_ref);

    // Record the command position and yaw for the command recorder
    Vector3Df command_uav_pos;
    command_uav_pos.x = pos_err.x;
    command_uav_pos.y = pos_err.y;
    command_uav_pos.z = 0.0f; // Assuming z is not used
    Quaternion command_uav_yaw(0, 0, 0, refAngles.yaw); // Assuming no rotation for simplicity
    commandRecorder->updatePosition(command_uav_pos, command_uav_yaw);

    refAngles.yaw=yaw_ref;

    uX->SetValues(pos_err.x, vel_err.x);
    uX->Update(GetTime());
    refAngles.pitch=uX->Output();

    uY->SetValues(pos_err.y, vel_err.y);
    uY->Update(GetTime());
    refAngles.roll=-uY->Output();

    customReferenceOrientation->SetQuaternionAndAngularRates(refAngles.ToQuaternion(),Vector3Df(0,0,0));

    return customReferenceOrientation;
}




void test_fleet::newpoint(void){
    path_indice+=1;
}

void test_fleet::timerrun() {
    while (running){
        if(test_fleet::Get_flag_followpath()){
            unsigned int microsecond = 1000000;
            usleep(microsecond/v->Value());//sleeps for second
            newpoint();
        }
    }
}     

void test_fleet::PositionValues(Vector2Df &pos_error,Vector2Df &vel_error,float &yaw_ref) {
    time=time+10.0; //+0.1s
    if (behaviourMode==BehaviourMode_t::FollowPathUAV){
        flag_followpath=true;
        commandServerUAV->validateCommands(false);
        line_detected=false;
    }
    else if (behaviourMode==BehaviourMode_t::UAVControlTCP){
        commandServerUAV->validateCommands(true);

    }
    else {
        flag_followpath=false;
        line_detected=false;
        commandServerUAV->validateCommands(false);
    }

    Vector3Df uav_pos,uav_vel; // in VRPN coordinate system
    Vector2Df uav_2Dpos,uav_2Dvel; // in VRPN coordinate system

    uavVrpn->GetPosition(uav_pos);
    uavVrpn->GetSpeed(uav_vel);

    uav_pos.To2Dxy(uav_2Dpos);
    uav_vel.To2Dxy(uav_2Dvel);

    Quaternion currentQuaternion=GetCurrentQuaternion();
    Euler currentAngles;//in vrpn frame
    currentQuaternion.ToEuler(currentAngles);
    float yaw_uav=currentAngles.yaw;


    if (behaviourMode==BehaviourMode_t::PositionHold) {
        pos_error=uav_2Dpos-posHold;
        vel_error=uav_2Dvel;
        yaw_ref=yawHold;
    }

    else if (behaviourMode==BehaviourMode_t::UAVControlTCP) {
        if (commandServerUAV->getCommand4() > 0.0) {
            pos_error.x=commandServerUAV->getCommand1();
            pos_error.y=commandServerUAV->getCommand2();
            vel_error=uav_2Dvel;
            yaw_ref=commandServerUAV->getCommand3();
        
            line_detected=true;
            yaw_uav=0.0;
        }
        else {
            line_detected=false;
        }

    }

    else if (behaviourMode==BehaviourMode_t::FollowPathUAV){

        if (path_init==false){   
            float distance = std::sqrt(std::pow(uav_2Dpos.x - pathUAV[0].first, 2) + 
                                        std::pow(uav_2Dpos.y - pathUAV[0].second, 2));
            float min_value = distance;
            int min_index = 0;
            
            for (int i = 1; i < path_length; i++) {
                distance = std::sqrt(std::pow(uav_2Dpos.x - pathUAV[i].first, 2) + 
                                     std::pow(uav_2Dpos.y - pathUAV[i].second, 2));               
                if (distance < min_value) {
                    min_value = distance;
                    min_index = i;
                }
            }
            path_indice=min_index;
            path_init = true;
        }

        if (path_indice==path_length){
            path_indice=0;
        }     

        posHold.x = pathUAV[path_indice].first;
        posHold.y = pathUAV[path_indice].second;
        
        matrix_path->SetValueNoMutex(0, 0, posHold.x);
        matrix_path->SetValueNoMutex(1, 0, posHold.y);

        if (std::fmod(time, v->Value()) == 0.0) {
            newpoint();
        }
        
        pos_error=uav_2Dpos-posHold;

        
        vel_error=uav_2Dvel;
        yaw_ref=atan2(posHold.y-uav_pos.y,posHold.x-uav_pos.x);

        if((yaw_uav > PI/2 && yaw_uav< PI)&&(yaw_ref < -PI/2 && yaw_ref > -PI)){
            yaw_ref += 2*PI;
        }else if((yaw_uav < -PI/2 && yaw_uav> -PI)&&(yaw_ref > PI/2 && yaw_ref < PI)){
            yaw_ref -=2*PI;
        }
    }

    else { //Circle
        Vector3Df target_pos;
        Vector2Df circle_pos,circle_vel;
        Vector2Df target_2Dpos;

        // targetVrpn->GetPosition(target_pos);
        target_pos.To2Dxy(target_2Dpos);
        circle->SetCenter(target_2Dpos);

        //circle reference
        circle->Update(GetTime());
        circle->GetPosition(circle_pos);
        circle->GetSpeed(circle_vel);

        //error in optitrack frame
        pos_error=uav_2Dpos-circle_pos;
        vel_error=uav_2Dvel-circle_vel;
        yaw_ref=atan2(target_pos.y-uav_pos.y,target_pos.x-uav_pos.x);

        if((yaw_uav > PI/2 && yaw_uav< PI)&&(yaw_ref < -PI/2 && yaw_ref > -PI)){
            yaw_ref += 2*PI;
        }else if((yaw_uav < -PI/2 && yaw_uav> -PI)&&(yaw_ref > PI/2 && yaw_ref < PI)){
            yaw_ref -=2*PI;
        }
        
    }
    float sat_yaw =PI/15;
    float diff = yaw_uav-yaw_ref;


        if (yaw_ref-yaw_uav>sat_yaw)  {
            yaw_ref=yaw_uav+sat_yaw;
        }
        else if (yaw_ref-yaw_uav<-sat_yaw)  {
            yaw_ref=yaw_uav-sat_yaw;
        }


    //error in uav frame
    currentQuaternion=GetCurrentQuaternion();
    currentQuaternion.ToEuler(currentAngles);
    pos_error.Rotate(-currentAngles.yaw);
    vel_error.Rotate(-currentAngles.yaw);

}

float test_fleet::ComputeCustomThrust(void)
{
    cout << thrust << endl;
    return thrust;
}

void test_fleet::SignalEvent(Event_t event) {
    UavStateMachine::SignalEvent(event);
    switch(event) {
    case Event_t::TakingOff:
        behaviourMode=BehaviourMode_t::Default;
        vrpnLost=false;

        // VrpnPositionHold();
        Thread::Warn("test_fleet: TakingOff \n");
        break;
    case Event_t::EnteringControlLoop:
        if ((behaviourMode==BehaviourMode_t::Circle) && (!circle->IsRunning())) {
            VrpnPositionHold();

        }
        break;
    case Event_t::EnteringFailSafeMode:
        behaviourMode=BehaviourMode_t::Default;
        break;
    }
}

void test_fleet::ExtraSecurityCheck(void) {
    if ((!vrpnLost) && ((behaviourMode==BehaviourMode_t::Circle) || (behaviourMode==BehaviourMode_t::PositionHold) || (behaviourMode==BehaviourMode_t::FollowPathUAV) || (behaviourMode==BehaviourMode_t::UAVControlTCP))) {//||(behaviourMode==BehaviourMode_t::StopFollowUGV))) {
        // if (!targetVrpn->IsTracked(500)) {
        //     Thread::Err("VRPN, target lost\n");
        //     vrpnLost=true;
        //     EnterFailSafeMode();
        //     Land();
        // }
        if (!uavVrpn->IsTracked(500)) {
            Thread::Err("VRPN, uav lost\n");
            vrpnLost=true;
            EnterFailSafeMode();
            Land();
        }
    }
}

void test_fleet::ExtraCheckPushButton(void) {
    if(startCircle->Clicked()) {
        StartCircle();
    }

    if(stopCircle->Clicked()) {
        StopCircle();

    }
    if(positionHold->Clicked()) {
        VrpnPositionHold();

    }
    if(followPathUAV->Clicked()){
        path_init = false;
        VrpnFollowPath();

    }
    if(UAVControlTCP->Clicked()){
        ControledByTCP();
    }
}

void test_fleet::ExtraCheckJoystick(void) {
    //                         Do not use cross, start nor select buttons!!
	//     0: "start"       1: "select"      2: "square"      3: "triangle"
	//     4: "circle"      5: "cross";      6: "left 1"      7: "left 2"
	//     8: "left 3"      9: "right 1"     10: "right 2"    11: "right 3"
	//     12: "up"         13: "down"       14: "left"       15: "right"    

    //R1 and Circle
    if(GetTargetController()->ButtonClicked(4) && GetTargetController()->ButtonClicked(9)) {
        StartCircle();

    }
    //Circle
    if(GetTargetController()->ButtonClicked(4) ){
        ControledByTCP();
    }

    //R1 and Triangle
    if(GetTargetController()->ButtonClicked(3) && GetTargetController()->ButtonClicked(9)) {
        path_init = false;
        VrpnFollowPath();
    }
    
    //R1 and square
    if(GetTargetController()->ButtonClicked(2) && GetTargetController()->ButtonClicked(9)) {
        VrpnPositionHold();
    }

}

void test_fleet::StartCircle(void) {
    if( behaviourMode==BehaviourMode_t::Circle) {
        Thread::Warn("test_fleet: already in circle mode\n");
        return;
    }
    if (SetOrientationMode(OrientationMode_t::Custom)) {
        Thread::Info("test_fleet: start circle\n");
    } else {
        Thread::Warn("test_fleet: could not start circle\n");
        return;
    }
    Vector3Df uav_pos,target_pos;
    Vector2Df uav_2Dpos,target_2Dpos;

    target_pos.To2Dxy(target_2Dpos);
    circle->SetCenter(target_2Dpos);

    uavVrpn->GetPosition(uav_pos);
    uav_pos.To2Dxy(uav_2Dpos);
    circle->StartTraj(uav_2Dpos);

    uX->Reset();
    uY->Reset();
    behaviourMode=BehaviourMode_t::Circle;
}

void test_fleet::StopCircle(void) {
    if( behaviourMode!=BehaviourMode_t::Circle) {
        Thread::Warn("test_fleet: not in circle mode\n");
        return;
    }
    circle->FinishTraj();
    Thread::Info("test_fleet: finishing circle\n");
}

void test_fleet::VrpnPositionHold(void) {
    if( behaviourMode==BehaviourMode_t::PositionHold) {
        Thread::Warn("test_fleet: already in vrpn position hold mode\n");
        return;
    }
	Quaternion vrpnQuaternion;
    uavVrpn->GetQuaternion(vrpnQuaternion);
		yawHold=vrpnQuaternion.ToEuler().yaw;

    Vector3Df vrpnPosition;
    uavVrpn->GetPosition(vrpnPosition);
    vrpnPosition.To2Dxy(posHold);

    uX->Reset();
    uY->Reset();
    behaviourMode=BehaviourMode_t::PositionHold;
    SetOrientationMode(OrientationMode_t::Custom);
    Thread::Info("test_fleet: holding position\n");
}

void test_fleet::ControledByTCP(void) {
    if( behaviourMode==BehaviourMode_t::UAVControlTCP) {
        Thread::Warn("test_fleet: already in TCP mode\n");
        return;
    }
    Quaternion vrpnQuaternion;
    uavVrpn->GetQuaternion(vrpnQuaternion);
        yawHold=vrpnQuaternion.ToEuler().yaw;

    Vector3Df vrpnPosition;
    uavVrpn->GetPosition(vrpnPosition);
    vrpnPosition.To2Dxy(posHold);

    uX->Reset();
    uY->Reset();
    behaviourMode=BehaviourMode_t::UAVControlTCP;
    SetOrientationMode(OrientationMode_t::Custom);
    Thread::Info("test_fleet: UAV controlled by TCP\n");
}

void test_fleet::VrpnFollowPath(void) {
    if( behaviourMode==BehaviourMode_t::FollowPathUAV) {
        Thread::Warn("test_fleet: already in vrpn follow path mode\n");
        return;
    }
	Quaternion vrpnQuaternion;
    uavVrpn->GetQuaternion(vrpnQuaternion);
		yawHold=vrpnQuaternion.ToEuler().yaw;

    Vector3Df vrpnPosition;
    uavVrpn->GetPosition(vrpnPosition);
    vrpnPosition.To2Dxy(posHold);

    uX->Reset();
    uY->Reset();
    uZ_custom->Reset();
    behaviourMode=BehaviourMode_t::FollowPathUAV;
    SetOrientationMode(OrientationMode_t::Custom);
    
    
    Thread::Info("test_fleet: Follow path\n");
}
