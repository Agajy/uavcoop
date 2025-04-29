//  created:    2011/05/01
//  filename:   test_fleet.cpp
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    demo cercle avec optitrack
//
//
/*********************************************************************/

#include "test_fleet.h"
#include "read_csv.h"
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

test_fleet::test_fleet(TargetController *controller): UavStateMachine(controller), behaviourMode(BehaviourMode_t::Default), vrpnLost(false), running(true) {
    Uav* uav=GetUav();

    VrpnClient* vrpnclient=new VrpnClient("vrpn", uav->GetDefaultVrpnAddress(),80,uav->GetDefaultVrpnConnectionType());
    
    if(vrpnclient->ConnectionType()==VrpnClient::Xbee) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName(),(uint8_t)0);
    } else if (vrpnclient->ConnectionType()==VrpnClient::Vrpn) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName());
    } else if (vrpnclient->ConnectionType()==VrpnClient::VrpnLite) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName());
    }
    
    // targetVrpn=new MetaVrpnObject("ugv_0");

    //set vrpn as failsafe altitude sensor for mamboedu as us in not working well for the moment
    if(uav->GetType()=="mamboedu") {
      SetFailSafeAltitudeSensor(uavVrpn->GetAltitudeSensor());
    }
    
    getFrameworkManager()->AddDeviceToLog(uavVrpn);
    // getFrameworkManager()->AddDeviceToLog(targetVrpn);
    vrpnclient->Start();
    
    uav->GetAhrs()->YawPlot()->AddCurve(uavVrpn->State()->Element(2),DataPlot::Green);
																 
    startCircle=new PushButton(GetButtonsLayout()->NewRow(),"Start_circle");
    stopCircle=new PushButton(GetButtonsLayout()->LastRowLastCol(),"stop_circle");
    // startFollowUGV=new PushButton(GetButtonsLayout()->NewRow(),"follow_ugv");
    // stopFollowUGV=new PushButton(GetButtonsLayout()->LastRowLastCol(),"stop_follow_ugv");
    positionHold=new PushButton(GetButtonsLayout()->LastRowLastCol(),"position hold");
    followPathUAV=new PushButton(GetButtonsLayout()->LastRowLastCol(),"follow path");

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

    // timerThread = std::thread(&test_fleet::timerrun, this);
    // timerThread = new TimerThread(this, "TimerThread");
    v = new DoubleSpinBox(GetButtonsLayout()->NewRow(), "step*s", " step*s", 20, 100, 0.01);


}

test_fleet::~test_fleet() {
    running = false;  // Signale au thread de s'arrêter
    // if (timerThread.joinable()) {
    //     timerThread.join();  // Attend la fin du thread
    // }
    // delete timerThread;
}


bool test_fleet::Get_flag_followpath(void){
    return flag_followpath;
}

const AhrsData *test_fleet::GetOrientation(void) const {
    //get yaw from vrpn
		Quaternion vrpnQuaternion;
    uavVrpn->GetQuaternion(vrpnQuaternion);

    //get roll, pitch and w from imu
    Quaternion ahrsQuaternion;
    Vector3Df ahrsAngularSpeed;
    GetDefaultOrientation()->GetQuaternionAndAngularRates(ahrsQuaternion, ahrsAngularSpeed);

    Euler ahrsEuler=ahrsQuaternion.ToEuler();
    ahrsEuler.yaw=vrpnQuaternion.ToEuler().yaw;
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
    }
    else {
        flag_followpath=false;
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

    // else if (behaviourMode==BehaviourMode_t::StopFollowUGV) {
    //     pos_error=uav_2Dpos-posHold2;
    //     vel_error=uav_2Dvel;
    //     yaw_ref=yawHold;
    // }
    // else if (behaviourMode==BehaviourMode_t::FollowUGV) {
    //     Vector3Df target_pos;
    //     Vector2Df target_2Dpos;

    //     targetVrpn->GetPosition(target_pos);
    //     target_pos.To2Dxy(target_2Dpos);

    //     pos_error=uav_2Dpos-target_2Dpos;
    //     vel_error=uav_2Dvel;
    //     yaw_ref=yawHold;
    // } 
    else if (behaviourMode==BehaviourMode_t::FollowPathUAV){
        // Vector3Df posHold_3D;
        // targetVrpn->GetPosition(posHold_3D);
        // Vector2Df uav_2Dpos(posHold_3D.x, posHold_3D.y);


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
    // cout << thrust << endl;
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
    if ((!vrpnLost) && ((behaviourMode==BehaviourMode_t::Circle) || (behaviourMode==BehaviourMode_t::PositionHold) || (behaviourMode==BehaviourMode_t::FollowPathUAV))) {//||(behaviourMode==BehaviourMode_t::StopFollowUGV))) {
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
    // if(startFollowUGV->Clicked()) {
    //     Thread::Info("test_fleet: start follow ugv\n");
    //     VrpnFollowUGV();
    // }
    // if(stopFollowUGV->Clicked()) {
    //     Thread::Info("test_fleet: stop follow ugv\n");
    //     VrpnStopFollowUGV();
    // }
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

    // targetVrpn->GetPosition(target_pos);
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
    //GetJoystick()->Rumble(0x70);
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

void test_fleet::VrpnFollowPath(void) {
    if( behaviourMode==BehaviourMode_t::FollowPathUAV) {
        Thread::Warn("test_fleet: already in vrpn follow path mode\n");
        return;
    }
    // if (SetThrustMode(ThrustMode_t::Custom))
    // {
    //     Thread::Info("test_fleet: already in custom thrust mode\n");
    // }
    // else
    // {
    //     Thread::Err("test_fleet: wrong\n");
    //     return;
    // }

    // path_indice=0;
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

// void test_fleet::VrpnFollowUGV(void) {
//     if( behaviourMode==BehaviourMode_t::FollowUGV) {
//         Thread::Warn("test_fleet: already in vrpn position hold mode\n");
//         return;
//     }
// 		Quaternion vrpnQuaternion;
//     uavVrpn->GetQuaternion(vrpnQuaternion);
// 		yawHold=vrpnQuaternion.ToEuler().yaw;

//     Vector3Df vrpnPosition;
//     uavVrpn->GetPosition(vrpnPosition);
//     vrpnPosition.To2Dxy(posHold);

//     uX->Reset();
//     uY->Reset();
//     behaviourMode=BehaviourMode_t::FollowUGV;
//     SetOrientationMode(OrientationMode_t::Custom);
//     Thread::Info("test_fleet: holding position\n");
// }


// void test_fleet::VrpnStopFollowUGV(void) {
//     if( behaviourMode==BehaviourMode_t::StopFollowUGV) {
//         Thread::Warn("test_fleet: already in vrpn position hold mode\n");
//         return;
//     }
// 		Quaternion vrpnQuaternion;
//     uavVrpn->GetQuaternion(vrpnQuaternion);
// 		yawHold=vrpnQuaternion.ToEuler().yaw;

//     Vector3Df vrpnPosition;
//     uavVrpn->GetPosition(vrpnPosition);
//     vrpnPosition.To2Dxy(posHold);

//     uX->Reset();
//     uY->Reset();
//     behaviourMode=BehaviourMode_t::StopFollowUGV;
//     SetOrientationMode(OrientationMode_t::Custom);
//     Thread::Info("test_fleet: holding position\n");
// }
