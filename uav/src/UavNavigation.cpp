//  created:    2011/05/01
//  filename:   UavNavigation.cpp
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

#include "UavNavigation.h"
#include <TargetController.h>
#include <Uav.h>
#include <GridLayout.h>
#include <PushButton.h>
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
#include <Ahrs.h>
#include <AhrsData.h>
#include <string.h>
// #include <CvtColor.h>
// #include "streaming/ServerTCP.h"
// #include <Camera.h> 
// #include <V4LCamera.h>
#include <UdpSocket.h>
#include <UgvControls.h>
#include <Ugv.h>

#define PI                  ((float)3.14159265358979323846)
#define UGV_READY           1
#define UGV_BUSY            2
#define UGV_END             3

#define TARGET_VISITED      2
#define TARGET_NOT_VISITED  -1

using namespace std;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::sensor;
using namespace flair::filter;
using namespace flair::meta;

UavNavigation::UavNavigation(TargetController *controller, const std::string streaminIp): UavStateMachine(controller), behaviourMode(BehaviourMode_t::Default), vrpnLost(false) {
    Uav* uav=GetUav();

    VrpnClient* vrpnclient=new VrpnClient("vrpn", uav->GetDefaultVrpnAddress(),80,uav->GetDefaultVrpnConnectionType());
    
    if(vrpnclient->ConnectionType()==VrpnClient::Xbee) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName(),(uint8_t)0);
        targetVrpn=new MetaVrpnObject("ugv",1);
    } else if (vrpnclient->ConnectionType()==VrpnClient::Vrpn) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName());
        targetVrpn=new MetaVrpnObject("ugv");
    } else if (vrpnclient->ConnectionType()==VrpnClient::VrpnLite) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName());
        targetVrpn=new MetaVrpnObject("ugv");
    }

    // CAMERA
    // if (uav->GetVerticalCamera() == NULL) {
    //     Err("no vertical camera found\n");
    //     exit(1);
    // }
    // image streaming
    this->server=new ServerTCP("server_communication",20003, streaminIp, uavVrpn, targetVrpn, &currDroneOrientation);
    this->server->Start();
    
    //set vrpn as failsafe altitude sensor for mamboedu as us in not working well for the moment
    if(uav->GetType()=="mamboedu") {
      SetFailSafeAltitudeSensor(uavVrpn->GetAltitudeSensor());
    }
    
    getFrameworkManager()->AddDeviceToLog(uavVrpn);
    // getFrameworkManager()->AddDeviceToLog(targetVrpn);
    vrpnclient->Start();
    
    uav->GetAhrs()->YawPlot()->AddCurve(uavVrpn->State()->Element(2),DataPlot::Green);

    // this->greyImageCameraVertical = new CvtColor(uav->GetVerticalCamera(),"gray",CvtColor::Conversion_t::ToGray);

    startCircle=new PushButton(GetButtonsLayout()->NewRow(),"start_circle");
    stopCircle=new PushButton(GetButtonsLayout()->LastRowLastCol(),"stop_circle");
    positionHold=new PushButton(GetButtonsLayout()->LastRowLastCol(),"position hold");

    uX=new Pid(setupLawTab->At(1,0),"u_x");
    uX->UseDefaultPlot(graphLawTab->NewRow());
    uY=new Pid(setupLawTab->At(1,1),"u_y");
    uY->UseDefaultPlot(graphLawTab->LastRowLastCol());

    customReferenceOrientation= new AhrsData(this,"reference");
    uav->GetAhrs()->AddPlot(customReferenceOrientation,DataPlot::Yellow);
    AddDataToControlLawLog(customReferenceOrientation);
    AddDeviceToControlLawLog(uX);
    AddDeviceToControlLawLog(uY);

    customOrientation=new AhrsData(this,"orientation");

    float dz_ref_;
    GetDefaultReferenceAltitude(desired_altitude, dz_ref_);
    altitude_error_integral = 0;

    // pos_error_prev.x = 0;
    // pos_error_prev.y = 0;
    message=new UdpSocket(uav,"Message","127.255.255.255:20010",true);
}

UavNavigation::~UavNavigation() {
}

const AhrsData *UavNavigation::GetOrientation(void) const {
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

void UavNavigation::AltitudeValues(float &z,float &dz) const{
    Vector3Df uav_pos,uav_vel;

    uavVrpn->GetPosition(uav_pos);
    uavVrpn->GetSpeed(uav_vel);
    //z and dz must be in uav's frame
    z=-uav_pos.z;
    dz=-uav_vel.z;
}

AhrsData *UavNavigation::GetReferenceOrientation(void) {
    // Vector2Df pos_err, vel_err; // in Uav coordinate system
    float yaw_ref;
    Euler refAngles;

    // flair::core::Time t1 = GetTime();
    // float dt = (t1-t0_)*1e-9;
    // if(dt>0.1){
    //     PositionValues(pos_err, vel_err, yaw_ref);
    //     t0_ = t1;
    //     }
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

// getReferanceAltitude fucntion
    // void UavNavigation::GetReferenceAltitude(float &refAltitude,
    //                                     float &refVerticalVelocity){
    //     Vector3Df uav_pos,uav_vel;
    //     uavVrpn->GetPosition(uav_pos);
    //     // uavVrpn->GetSpeed(uav_vel);
    //     float z = -uav_pos.z;
    //     // float z_dot = uav_vel.z;
    //
    //     Vector3Df data_pos;
    //     float data_yaw;
    //     this->server->getUgvPos(data_pos, data_yaw);
    //
    //     refAltitude = data_pos.z;//1.8;
    //     if (refAltitude < 1.5)
    //         refAltitude = 1.5;
    //     else if (refAltitude > 2.5)
    //         refAltitude = 2.5;
    //     refVerticalVelocity = 0;
    //
    //     if (!line_tracking_started)
    //         refAltitude = 2.5;
    //      
    //     //========= ALTITUDE HIGH LEVEL CONTROLLER =================
    //     // flair::core::Time t1 = GetTime();
    //     // float dt = (t1-t0)*1e-9;
    //     // t0 = t1;
    //
    //     // if(subindo)
    //     //     desired_altitude += 0.1*dt;
    //     // else
    //     //     desired_altitude -= 0.1*dt;
    //     // if (desired_altitude>1.5){
    //     //     subindo = false;
    //     //     desired_altitude = 1.5;}
    //     // if (desired_altitude<1){
    //     //     subindo = true;
    //     //     desired_altitude = 1;}
    //
    //     // float error_z = desired_altitude - z;
    //     // float sat = 0.13;
    //     // if (std::abs(error_z)>sat)
    //     //     error_z = sat*error_z/std::abs(error_z);
    //
    //     // refAltitude = z + error_z;//desired_altitude;
    //     // float k1=1.0, k2=0.0;
    //     // refVerticalVelocity = k1*error_z + k2*altitude_error_integral;//desired_altitude*0.1/std::abs(desired_altitude);
    //     // if (std::abs(refVerticalVelocity)>0.1)
    //     //     refVerticalVelocity = 0.1*refVerticalVelocity/std::abs(refVerticalVelocity);
    //
    //     // altitude_error_integral += (z - refAltitude)*dt;
    //     // if (std::abs(altitude_error_integral)>1)
    //     //     altitude_error_integral = 1*altitude_error_integral/std::abs(altitude_error_integral);
    //     //==========================================================
    //
    //     // std::cout << "MSG: " << z << ", " << refAltitude << ", " << error_z << ", " << dt << std::endl;
    // }

void UavNavigation::PositionValues(Vector2Df &pos_error,Vector2Df &vel_error,float &yaw_ref) {
    Vector3Df uav_pos,uav_vel; // in VRPN coordinate system
    Vector2Df uav_2Dpos,uav_2Dvel; // in VRPN coordinate system

    uavVrpn->GetPosition(uav_pos);
    uavVrpn->GetSpeed(uav_vel);

    uav_pos.To2Dxy(uav_2Dpos);
    uav_vel.To2Dxy(uav_2Dvel);

    // get UAV current angle
    Quaternion currentQuaternion=GetCurrentQuaternion();
    Euler currentAngles;//in vrpn frame
    currentQuaternion.ToEuler(currentAngles);

    if (behaviourMode==BehaviourMode_t::PositionHold) {
        pos_error=uav_2Dpos-posHold;
        vel_error=uav_2Dvel;
        yaw_ref=yawHold;
        //error in uav frame
        pos_error.Rotate(-currentAngles.yaw);
        vel_error.Rotate(-currentAngles.yaw);
    } else if(behaviourMode==BehaviourMode_t::Circle){ //Circle
        Vector3Df target_pos, target_vel;
        Vector2Df target_2Dpos, target_2Dvel;
        Vector2Df desired_2Dpos, desired_2Dvel; // in VRPN coordinate system
        Quaternion target_quat, uav_quat;

        targetVrpn->GetPosition(target_pos);
        targetVrpn->GetSpeed(target_vel);
        targetVrpn->GetQuaternion(target_quat);
        uavVrpn->GetQuaternion(uav_quat);
        target_pos.To2Dxy(target_2Dpos);
        target_vel.To2Dxy(target_2Dvel);

        Euler target_euler, uav_euler;
        target_quat.ToEuler(target_euler);
        uav_quat.ToEuler(uav_euler);

        //^ X
        //|
        // ---> y
        // ground station here
        // IMPORTANT: CREATE OBJECT
        // UAV: place uav HEADING TO X, then rotate 90 degrees in yaw (motive software) 
        // UGV: place ugv HEADING TO X, then rotate 90 degrees in yaw (motive software)         

        // Get error from visual features (ground station msg)
        Vector3Df data_pos;
        float data_yaw;
        float ugv_acc, ugv_steering;
        bool valid;
        this->server->getUgvPos(data_pos, data_yaw, ugv_acc, ugv_steering, valid);

        desired_2Dpos.x = target_pos.x;// + std::cos(uav_euler.yaw)*0.09 + 0.17;//xs[index];
        desired_2Dpos.y = target_pos.y;// + std::sin(uav_euler.yaw)*0.09 + 0.17;//ys[index];
        desired_2Dvel.x = target_vel.x;
        desired_2Dvel.y = target_vel.y;
        yaw_ref = 0;

        if (valid && !line_tracking_started){
            line_tracking_started = true;
            t0_xy = GetTime();
        }

        // choose an option. 0: follow a path, 1: visual servoing, 2: follow ugv
        int option = 0;

        if (option==0 && line_tracking_started){
            flair::core::Time t1 = GetTime();
            float dt = (t1-t0_xy)*1e-9;
        
            float des_vel = data_pos.x;
            trajectory.get_point(t1, desired_2Dpos, desired_2Dvel, yaw_ref, des_vel);
            pos_error=uav_2Dpos-desired_2Dpos;
            vel_error=uav_2Dvel-desired_2Dvel;
            pos_error.Rotate(-currentAngles.yaw);
            vel_error.Rotate(-currentAngles.yaw);
        }
        // line_tracking_started = false;
        else if (option == 1 && line_tracking_started){// && dt>0.01){

            flair::core::Time t1 = GetTime();
            t0_xy = t1;
                
            float k1=1, k2=0;
            pos_error.x = k1*data_pos.x;
            pos_error.y = k1*data_pos.y;
            desired_2Dvel.x = 0;
            desired_2Dvel.y = 0;
            
            vel_error=uav_2Dvel-desired_2Dvel;
            //error in uav frame
            vel_error.Rotate(-currentAngles.yaw);
            }
        else{
            pos_error=uav_2Dpos-desired_2Dpos;
            vel_error=uav_2Dvel-desired_2Dvel;
            //error in uav frame
            pos_error.Rotate(-currentAngles.yaw);
            vel_error.Rotate(-currentAngles.yaw);
            }

        // send ugv commands to ugv simulator
        message->SendMessage(to_string(ugv_acc));
        message->SendMessage(to_string(ugv_steering));
    }
}

void UavNavigation::SignalEvent(Event_t event) {
    UavStateMachine::SignalEvent(event);
    switch(event) {
    case Event_t::TakingOff:
        behaviourMode=BehaviourMode_t::Default;
        vrpnLost=false;
        break;
    case Event_t::EnteringControlLoop:
        if ((behaviourMode==BehaviourMode_t::Circle) && !isRunning){//(!circle->IsRunning())) {
            VrpnPositionHold();
        }
        if(start_circle){
            StartCircle();
            this->start_circle = false;
        }
        break;
    case Event_t::EnteringFailSafeMode:
        behaviourMode=BehaviourMode_t::Default;
        break;
    }
}

void UavNavigation::ExtraSecurityCheck(void) {
    if ((!vrpnLost) && ((behaviourMode==BehaviourMode_t::Circle) || (behaviourMode==BehaviourMode_t::PositionHold))) {
        if (!targetVrpn->IsTracked(500)) {
            Thread::Err("VRPN, target lost\n");
            vrpnLost=true;
            EnterFailSafeMode();
            Land();
        }
        if (!uavVrpn->IsTracked(500)) {
            Thread::Err("VRPN, uav lost\n");
            vrpnLost=true;
            EnterFailSafeMode();
            Land();
        }
    }
}

void UavNavigation::ExtraCheckPushButton(void) {
    if(startCircle->Clicked() && (behaviourMode!=BehaviourMode_t::Circle)) {
        StartCircle();
    }
    if(stopCircle->Clicked() && (behaviourMode==BehaviourMode_t::Circle)) {
        StopCircle();
    }
    if(positionHold->Clicked() && (behaviourMode==BehaviourMode_t::Default)) {
        VrpnPositionHold();
    }
}

void UavNavigation::ExtraCheckJoystick(void) {
    //R1 and Circle
    if(GetTargetController()->IsButtonPressed(9) && GetTargetController()->IsButtonPressed(4) && (behaviourMode!=BehaviourMode_t::Circle)) {
        StartCircle();
    }

    //R1 and Cross
    if(GetTargetController()->IsButtonPressed(9) && GetTargetController()->IsButtonPressed(5) && (behaviourMode==BehaviourMode_t::Circle)) {
        StopCircle();
    }
    
    //R1 and Square
    if(GetTargetController()->IsButtonPressed(9) && GetTargetController()->IsButtonPressed(2) && (behaviourMode==BehaviourMode_t::Default)) {
        VrpnPositionHold();
    }
}

void UavNavigation::StartCircle(void) {
    if (SetOrientationMode(OrientationMode_t::Custom)) {
        Thread::Info("UavNavigation: start circle\n");
    } else {
        Thread::Warn("UavNavigation: could not start circle\n");
        return;
    }
    trajectory.reset();

    uX->Reset();
    uY->Reset();
    behaviourMode=BehaviourMode_t::Circle;

    // SetAltitudeMode(AltitudeMode_t::Custom);
    t0 = GetTime();
    t0_ = GetTime();
    line_tracking_started = false;
    this->server->resetUgvPos();

    isRunning = true;
}

void UavNavigation::StopCircle(void) {
    // circle->FinishTraj();
    //GetJoystick()->Rumble(0x70);
    Thread::Info("UavNavigation: finishing circle\n");
    // behaviourMode=BehaviourMode_t::Default;
    // isRunning = false;

    Quaternion vrpnQuaternion;
    uavVrpn->GetQuaternion(vrpnQuaternion);
		yawHold=vrpnQuaternion.ToEuler().yaw;

    Vector3Df vrpnPosition;
    uavVrpn->GetPosition(vrpnPosition);
    vrpnPosition.To2Dxy(posHold);
    float modulo = std::sqrt(posHold.x*posHold.x + posHold.y*posHold.y);
    posHold.x += (0-posHold.x)*0.80/modulo;
    posHold.y += (0-posHold.y)*0.80/modulo;

    uX->Reset();
    uY->Reset();
    behaviourMode=BehaviourMode_t::PositionHold;
    SetOrientationMode(OrientationMode_t::Custom);
    Thread::Info("UavNavigation: holding position\n");
    isRunning = false;
}

void UavNavigation::VrpnPositionHold(void) {
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
    Thread::Info("UavNavigation: holding position\n");
    isRunning = false;
}
