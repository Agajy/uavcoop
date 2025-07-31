//  created:    2020/12/21
//  filename:   UgvNavigation.cpp
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    demo simple fleet avec optitrack
//
//
/*********************************************************************/

#include "UgvNavigation.h"
#include <TargetController.h>
#include <GridLayout.h>
#include <PushButton.h>
#include <DataPlot1D.h>
#include <DataPlot2D.h>
#include <FrameworkManager.h>
#include <VrpnClient.h>
#include <MetaVrpnObject.h>
#include <TrajectoryGenerator2DCircle.h>
#include <Matrix.h>
#include <Tab.h>
#include <TabWidget.h>
#include <DoubleSpinBox.h>
#include <Pid.h>
#include <Quaternion.h>
#include <Euler.h>
#include <Ugv.h>
#include <UgvControls.h>
#include <math.h>
#include <UdpSocket.h>
#include <string.h>
#include <iostream>

using namespace std;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::sensor;
using namespace flair::filter;
using namespace flair::meta;
using namespace flair::actuator;

UgvNavigation::UgvNavigation(string name,TargetController *controller): Thread(getFrameworkManager(),"UgvNavigation",50), behaviourMode(BehaviourMode_t::Manual), vrpnLost(false) {
    this->controller=controller;
    controller->Start();
    
    Ugv* ugv=GetUgv();
    ugv->UseDefaultPlot();
    
    VrpnClient* vrpnclient=new VrpnClient("vrpn", ugv->GetDefaultVrpnAddress(),80);
    ugvVrpn = new MetaVrpnObject(name);
        
    getFrameworkManager()->AddDeviceToLog(ugvVrpn);
    vrpnclient->Start();
	
    Tab *ugvTab = new Tab(getFrameworkManager()->GetTabWidget(), "ugv", 0);
    GridLayout* buttonslayout = new GridLayout(ugvTab->NewRow(), "buttons");
    button_kill = new PushButton(buttonslayout->NewRow(), "kill");
    startCircle=new PushButton(buttonslayout->NewRow(),"start_circle");
    stopCircle=new PushButton(buttonslayout->LastRowLastCol(),"stop_circle");
    stop=new PushButton(buttonslayout->LastRowLastCol(),"stop");
    tcpCommand=new PushButton(buttonslayout->NewRow(),"TCP_Command");


    button_start_log = new PushButton(buttonslayout->NewRow(), "start_log");
    button_stop_log = new PushButton(buttonslayout->LastRowLastCol(), "stop_log");
    
    circle=new TrajectoryGenerator2DCircle(vrpnclient->GetLayout()->NewRow(),"circle");
    ugvVrpn->xPlot()->AddCurve(circle->GetMatrix()->Element(0,0),DataPlot::Blue);
    ugvVrpn->yPlot()->AddCurve(circle->GetMatrix()->Element(0,1),DataPlot::Blue);
    ugvVrpn->VxPlot()->AddCurve(circle->GetMatrix()->Element(1,0),DataPlot::Blue);
    ugvVrpn->VyPlot()->AddCurve(circle->GetMatrix()->Element(1,1),DataPlot::Blue);
    ugvVrpn->XyPlot()->AddCurve(circle->GetMatrix()->Element(0,1),circle->GetMatrix()->Element(0,0),DataPlot::Blue,"circle");
    
    xCircleCenter=new DoubleSpinBox(vrpnclient->GetLayout()->NewRow(),"x circle center"," m",-5,5,0.1,1,0);
    yCircleCenter=new DoubleSpinBox(vrpnclient->GetLayout()->NewRow(),"y circle center"," m",-5,5,0.1,1,0);

    Tab *lawTab = new Tab(getFrameworkManager()->GetTabWidget(), "control laws");
    TabWidget *tabWidget = new TabWidget(lawTab->NewRow(), "laws");
    Tab *setupLawTab = new Tab(tabWidget, "Setup");
    Tab *graphLawTab = new Tab(tabWidget, "Graphes");
    uX=new Pid(setupLawTab->At(1,0),"u_x");
    uX->UseDefaultPlot(graphLawTab->NewRow());
    uY=new Pid(setupLawTab->At(1,1),"u_y");
    uY->UseDefaultPlot(graphLawTab->LastRowLastCol());
    
    getFrameworkManager()->AddDeviceToLog(uX);
    getFrameworkManager()->AddDeviceToLog(uY);
    
    l=new DoubleSpinBox(setupLawTab->NewRow(),"L", " m", 0, 10, 0.1, 1,1);
    
    message=new UdpSocket(ugv,"Message","127.255.255.255:20010",true);
    acc = 0; 
    steering = 0;

    // Création du serveur reception command
    commandServerUGV = new CommandServerUGV(nullptr,name,"127.0.0.1","62736");

    // Initialiser le serveur avant de démarrer le thread
    if (commandServerUGV->initialize()) {
        // Configuration du callback si nécessaire
        commandServerUGV->setCommandCallback([](float v1, float v2) {
            std::cout << "Received commands: " << v1 << ", " << v2<< std::endl;
        });
    
        // Démarrer le thread seulement si l'initialisation a réussi
        commandServerUGV->Start();
    } else {
        std::cerr << "Failed to initialize command server" << std::endl;
    }
}

UgvNavigation::~UgvNavigation() {
    commandServerUGV->Join();
    delete commandServerUGV;
}

void UgvNavigation::Run(void) {
    WarnUponSwitches(true);
    SetPeriodMS(20);
    
    if (getFrameworkManager()->ErrorOccured() == true) {
        SafeStop();
    }

    while (!ToBeStopped()) {
        SecurityCheck();
        CheckMessages();
        CheckJoystick();
        CheckPushButton();
        
        if(behaviourMode==BehaviourMode_t::Manual) {
            ComputeManualControls();
            commandServerUGV->validateCommands(false);
        }
        else if(behaviourMode==BehaviourMode_t::Circle) {
            ComputeCircleControls();
            commandServerUGV->validateCommands(false);
        }
        else if(behaviourMode==BehaviourMode_t::Receive){
            ComputeReceiveControls();
            commandServerUGV->validateCommands(false);
        }
        else if(behaviourMode==BehaviourMode_t::TCPCommand) {
            ComputeTCPCommands();
            commandServerUGV->validateCommands(true);
        }
        else {
            behaviourMode=BehaviourMode_t::Default;
            Defaultmode();
            commandServerUGV->validateCommands(false);

        }
        WaitPeriod();
    }
}

void UgvNavigation::CheckPushButton(void) {
  if (button_start_log->Clicked() == true)
    getFrameworkManager()->StartLog();
  if (button_stop_log->Clicked() == true)
    getFrameworkManager()->StopLog();
    
  if (startCircle->Clicked() == true)
      StartCircle();

  if (stopCircle->Clicked() == true)
      StopCircle();

  if (stop->Clicked() == true)
      Defaultmode();

  if (button_kill->Clicked() == true)
      SafeStop();
    
  if (tcpCommand->Clicked() == true)
      TCPCommand();
}

void UgvNavigation::CheckJoystick(void) {
  //R1 and Circle
  if(controller->IsButtonPressed(9) && controller->IsButtonPressed(4)) {
      StartCircle();
  }

  //R1 and Cross
  if(controller->IsButtonPressed(9) && controller->IsButtonPressed(5)) {
      StopCircle();
  }
}

void UgvNavigation::SecurityCheck(void) {
    if ((!vrpnLost) && (behaviourMode==BehaviourMode_t::Circle)) {
        if (!ugvVrpn->IsTracked(500)) {
            Thread::Err("VRPN, ugv lost\n");
            vrpnLost=true;
            StopCircle();
        }
    }
}

void UgvNavigation::CheckMessages(void) {
    char msg[64];
    char src[64];
    size_t src_size=sizeof(src);
    // while(message->RecvMessage(msg,sizeof(msg),TIME_NONBLOCK,src,&src_size)>0) {
        //printf("%s %s\n",GetUav()->ObjectName().c_str(),src);
    if (message->RecvMessage(msg,sizeof(msg),TIME_NONBLOCK,src,&src_size)>0){
        if(strcmp(src,"x4_0")==0) {
            acc = atof (msg);
            std::cout << msg << std::endl;
        }
    }
    if (message->RecvMessage(msg,sizeof(msg),TIME_NONBLOCK,src,&src_size)>0){
        if(strcmp(src,"x4_0")==0) {
            steering = atof (msg);
            std::cout << msg << std::endl;
        }
    }
}

void UgvNavigation::ComputeReceiveControls(void) {
    GetUgv()->GetUgvControls()->SetControls(acc,steering);
}

void UgvNavigation::Defaultmode(void) {
    GetUgv()->GetUgvControls()->SetControls(0.0,0.0);
}

void UgvNavigation::ComputeManualControls(void) {
    float speed=-controller->GetAxisValue(3);
    float turn=controller->GetAxisValue(0);
    GetUgv()->GetUgvControls()->SetControls(speed,turn);
  }

void UgvNavigation::ComputeTCPCommands(void) {
    Vector2Df pos_error , vel_error;
    pos_error.x = commandServerUGV->getCommand1();
    pos_error.y = commandServerUGV->getCommand2();

    uX->SetValues(pos_error.x, vel_error.x);
    uX->Update(GetTime());
    uY->SetValues(pos_error.y, vel_error.y);
    uY->Update(GetTime());

    Quaternion vrpnQuaternion;
    ugvVrpn->GetQuaternion(vrpnQuaternion);
    float yaw=vrpnQuaternion.ToEuler().yaw;
    float L=1;
    float v= cosf(yaw)*uX->Output() + sinf(yaw)*uY->Output();
    float w = -sinf(yaw)/l->Value()*uX->Output() + cosf(yaw)/l->Value()*uY->Output();
    GetUgv()->GetUgvControls()->SetControls(-v,-w);
    }

void UgvNavigation::ComputeCircleControls(void) {

  Vector3Df ugv_pos,ugv_vel; // in VRPN coordinate system
  Vector2Df ugv_2Dpos,ugv_2Dvel,target_2Dpos; // in VRPN coordinate system
  Vector2Df pos_error,vel_error;
  Vector2Df circle_pos,circle_vel;
    
  ugvVrpn->GetPosition(ugv_pos);
  ugvVrpn->GetSpeed(ugv_vel);

  ugv_pos.To2Dxy(ugv_2Dpos);
  ugv_vel.To2Dxy(ugv_2Dvel);
  
  target_2Dpos.x=xCircleCenter->Value();
  target_2Dpos.y=yCircleCenter->Value();
  circle->SetCenter(target_2Dpos);

  //circle reference
  circle->Update(GetTime());
  circle->GetPosition(circle_pos);
  circle->GetSpeed(circle_vel);

  //error in optitrack frame
  pos_error=ugv_2Dpos-circle_pos;
  vel_error=ugv_2Dvel-circle_vel;
    
  uX->SetValues(pos_error.x, vel_error.x);
  uX->Update(GetTime());
  uY->SetValues(pos_error.y, vel_error.y);
  uY->Update(GetTime());
  
  //get yaw from vrpn
  Quaternion vrpnQuaternion;
  ugvVrpn->GetQuaternion(vrpnQuaternion);
  float yaw=vrpnQuaternion.ToEuler().yaw;
  float L=1;
  float v= cosf(yaw)*uX->Output() + sinf(yaw)*uY->Output();
  float w = -sinf(yaw)/l->Value()*uX->Output() + cosf(yaw)/l->Value()*uY->Output();
  GetUgv()->GetUgvControls()->SetControls(-v,-w);
}


void UgvNavigation::StartCircle(void) {
  if(behaviourMode!=BehaviourMode_t::Circle) {
    Vector3Df ugv_pos;
    Vector2Df ugv_2Dpos,target_2Dpos;

    target_2Dpos.x=xCircleCenter->Value();
    target_2Dpos.y=yCircleCenter->Value();
    circle->SetCenter(target_2Dpos);

    ugvVrpn->GetPosition(ugv_pos);
    ugv_pos.To2Dxy(ugv_2Dpos);
    circle->StartTraj(ugv_2Dpos);

    uX->Reset();
    uY->Reset();
    behaviourMode=BehaviourMode_t::Circle;
    Thread::Info("UgvNavigation: start circle\n");
    // message->SendMessage("StartCircle");
  }
}

void UgvNavigation::TCPCommand() {
    if(behaviourMode!=BehaviourMode_t::TCPCommand) {
        Vector3Df ugv_pos;
        Vector2Df ugv_2Dpos,target_2Dpos;
        commandServerUGV->validateCommands(true);

        ugvVrpn->GetPosition(ugv_pos);
        ugv_pos.To2Dxy(ugv_2Dpos);

        uX->Reset();
        uY->Reset();
        behaviourMode=BehaviourMode_t::TCPCommand;
        Thread::Info("UgvNavigation: TCPCommand\n");
  }
}

void UgvNavigation::StopCircle(void) {
    if(behaviourMode==BehaviourMode_t::Circle) {
      circle->FinishTraj();
      //GetJoystick()->Rumble(0x70);
      behaviourMode=BehaviourMode_t::Manual;
      Thread::Info("UgvNavigation: finishing circle\n");
    //   message->SendMessage("StopCircle");
    }
}

