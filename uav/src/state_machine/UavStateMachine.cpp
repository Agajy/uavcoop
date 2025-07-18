// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2014/04/29
//  filename:   UavStateMachine.cpp
//
//  author:     Gildas Bayard, Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    meta class for UAV
//
//
/*********************************************************************/

#include "UavStateMachine.h"
#include "Uav.h"
#include <DataPlot1D.h>
#include <GridLayout.h>
#include <Tab.h>
#include <TabWidget.h>
#include <PushButton.h>
#include <SpinBox.h>
#include <DoubleSpinBox.h>
#include <X4X8Multiplex.h>
#include <Bldc.h>
#include <Ahrs.h>
#include <MetaUsRangeFinder.h>
#include <ControlLaw.h>
#include <Pid.h>
#include <PidThrust.h>
#include <NestedSat.h>
#include <MetaDualShock3.h>
#include <AhrsData.h>
#include <BatteryMonitor.h>
#include <FrameworkManager.h>
#include <Vector3D.h>
#include <Vector2D.h>
#include <Matrix.h>
#include <stdio.h>
#include <TrajectoryGenerator1D.h>
#include <math.h>

using namespace std;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::sensor;
using namespace flair::actuator;
using namespace flair::filter;
using namespace flair::meta;

UavStateMachine::UavStateMachine(TargetController *controller):
  Thread(getFrameworkManager(),"UavStateMachine",50),
  uav(GetUav()),controller(controller),failSafeMode(true),flagConnectionLost(false),flagBatteryLow(false),flagCriticalSensorLost(false),flagZTrajectoryFinished(false),safeToFly(true){
  altitudeState=AltitudeState_t::Stopped;
  uav->UseDefaultPlot();

  Tab *uavTab = new Tab(getFrameworkManager()->GetTabWidget(), "uav", 0);
  buttonslayout = new GridLayout(uavTab->NewRow(), "buttons");
  button_quit = new PushButton(buttonslayout->NewRow(), "quit program");
  button_start_log = new PushButton(buttonslayout->NewRow(), "start_log");
  button_stop_log = new PushButton(buttonslayout->LastRowLastCol(), "stop_log");
  button_take_off = new PushButton(buttonslayout->NewRow(), "take_off");
  button_land = new PushButton(buttonslayout->LastRowLastCol(), "land");

  Tab *lawTab = new Tab(getFrameworkManager()->GetTabWidget(), "control laws");
  TabWidget *tabWidget = new TabWidget(lawTab->NewRow(), "laws");
  setupLawTab = new Tab(tabWidget, "Setup");
  graphLawTab = new Tab(tabWidget, "Graphes");

  uRoll = new NestedSat(setupLawTab->At(0, 0), "u_roll");
  uRoll->ConvertSatFromDegToRad();
  uRoll->UseDefaultPlot(graphLawTab->NewRow());

  uPitch = new NestedSat(setupLawTab->At(0, 1), "u_pitch");
  uPitch->ConvertSatFromDegToRad();
  uPitch->UseDefaultPlot(graphLawTab->LastRowLastCol());

  uYaw = new Pid(setupLawTab->At(0, 2), "u_yaw");
  uYaw->UseDefaultPlot(graphLawTab->LastRowLastCol());

  uZ = new PidThrust(setupLawTab->At(1, 2), "u_z");
  uZ->UseDefaultPlot(graphLawTab->LastRowLastCol());

  getFrameworkManager()->AddDeviceToLog(uZ);
  uZ->AddDeviceToLog(uRoll);
  uZ->AddDeviceToLog(uPitch);
  uZ->AddDeviceToLog(uYaw);

  joy=new MetaDualShock3("uav high level controller",controller);
  uav->GetAhrs()->AddPlot(joy->GetReferenceOrientation(),DataPlot::Blue);

  altitudeMode = AltitudeMode_t::Manual;
  orientationMode = OrientationMode_t::Manual;
  thrustMode = ThrustMode_t::Default;
  torqueMode = TorqueMode_t::Default;

  GroupBox *reglagesGroupbox =
      new GroupBox(uavTab->NewRow(), "takeoff/landing");
  desiredTakeoffAltitude =
      new DoubleSpinBox(reglagesGroupbox->NewRow(), "desired takeoff altitude",
                        " m", 0, 5, 0.1, 2, 1);
  desiredLandingAltitude =
      new DoubleSpinBox(reglagesGroupbox->LastRowLastCol(),
                        "desired landing altitude", " m", 0, 1, 0.1, 1);
  altitudeTrajectory =
      new TrajectoryGenerator1D(uavTab->NewRow(), "alt cons", "m");
  uav->GetMetaUsRangeFinder()->GetZPlot()->AddCurve(
      altitudeTrajectory->GetMatrix()->Element(0), DataPlot::Green);
  uav->GetMetaUsRangeFinder()->GetVzPlot()->AddCurve(
      altitudeTrajectory->GetMatrix()->Element(1), DataPlot::Green);
      
  failSafeAltitudeSensor=uav->GetMetaUsRangeFinder();
}

UavStateMachine::~UavStateMachine() {}

void UavStateMachine::AddDeviceToControlLawLog(const IODevice *device) {
  uZ->AddDeviceToLog(device);
}

void UavStateMachine::AddDataToControlLawLog(const core::io_data *data) {
  uZ->AddDataToLog(data);
}

TargetController *UavStateMachine::GetTargetController(void) const{
  return controller;
}

MetaDualShock3 *UavStateMachine::GetJoystick(void) const {
  return joy;
}

const Quaternion &UavStateMachine::GetCurrentQuaternion(void) const {
  return currentQuaternion;
}

const Vector3Df &UavStateMachine::GetCurrentAngularSpeed(void) const {
  return currentAngularSpeed;
}

void UavStateMachine::SetFailSafeAltitudeSensor(AltitudeSensor *altitudeSensor) {
  Warn("Replacing the FailSafeAltitudeSensor with %s\n",altitudeSensor->ObjectName().c_str());
  failSafeAltitudeSensor=altitudeSensor;
}

void UavStateMachine::AltitudeValues(float &altitude,
                                     float &verticalSpeed) const {
  FailSafeAltitudeValues(altitude, verticalSpeed);
}

void UavStateMachine::FailSafeAltitudeValues(float &altitude,
                                             float &verticalSpeed) const {
  altitude = failSafeAltitudeSensor->z();
  verticalSpeed = failSafeAltitudeSensor->Vz();
  
  //a mettre dans le metausrangefinder?
  //on ne sait pas si failSafeAltitudeSensor est vrpn (dans ce cas pas besoin de rotation)
  //ou un us range finder
  //+0.04 decalage en z de l'us en simulation
  //faire un reglage
  /*
  Vector3Df test(0,0,altitude+0.04);
  Printf("altitude av %f %f %f %f\n",altitude,test.x,test.y,test.z);
  test.Rotate(currentQuaternion);
  Printf("altitude ap %f %f %f %f\n",altitude,test.x,test.y,test.z-0.04);
   */
}

void UavStateMachine::Run() {
  WarnUponSwitches(true);
  uav->StartSensors();

  if (getFrameworkManager()->ErrorOccured() == true) {
    SafeStop();
  }

  while (!ToBeStopped()) {
    SecurityCheck();

    // get controller inputs
    CheckJoystick();
    CheckPushButton();

    if (IsPeriodSet()) {
      WaitPeriod();
    } else {
      WaitUpdate(uav->GetAhrs());
    }
    needToComputeDefaultTorques = true;
    needToComputeDefaultThrust = true;

    SignalEvent(Event_t::EnteringControlLoop);

    ComputeOrientation();
    ComputeAltitude();

    // compute thrust and torques to apply
    ComputeTorques();
    ComputeThrust(); // logs are added to uz, so it must be updated at last

    //check nan/inf problems
    bool isValuePossible=true;
    //test it separately to warn for all problems
    if(!IsValuePossible(currentTorques.roll,"roll torque")) isValuePossible=false;
    if(!IsValuePossible(currentTorques.pitch,"pitch torque")) isValuePossible=false;
    if(!IsValuePossible(currentTorques.yaw,"yaw torque")) isValuePossible=false;
    if(!IsValuePossible(currentThrust,"thrust")) isValuePossible=false;
    if(!isValuePossible) {

        if(altitudeState==AltitudeState_t::Stopped) {
            SafeStop();
        } else {

          if(failSafeMode) {
            Warn("We are already in safe mode, the uav is going to crash!\n");
          } else {
            Thread::Warn("switching back to safe mode\n");
            EnterFailSafeMode();
            needToComputeDefaultTorques = true;//should not be necessary, but put it to be sure to compute default thrust/torques
            needToComputeDefaultThrust = true;

            ComputeTorques();
            ComputeThrust();
          }
        }
    }

    // Set torques for roll, pitch and yaw angles (value between -1 and 1). Set
    // thrust (value between 0 and 1)
    uav->GetUavMultiplex()->SetRoll(-currentTorques.roll);
    uav->GetUavMultiplex()->SetPitch(-currentTorques.pitch);
    uav->GetUavMultiplex()->SetYaw(-currentTorques.yaw);
    uav->GetUavMultiplex()->SetThrust(-currentThrust); // on raisonne en negatif
                                                       // sur l'altitude, a
                                                       // revoir avec les
                                                       // equations
    uav->GetUavMultiplex()->SetRollTrim(joy->RollTrim());
    uav->GetUavMultiplex()->SetPitchTrim(joy->PitchTrim());
    uav->GetUavMultiplex()->SetYawTrim(0);
    uav->GetUavMultiplex()->Update(GetTime());
  }

  WarnUponSwitches(false);
}

bool UavStateMachine::IsValuePossible(float value,std::string desc) {
  if(isnan(value)) {
    Warn("%s is not an number\n",desc.c_str());
    return false;
  } else if(isinf(value)) {
    Warn("%s is infinite\n",desc.c_str());
    return false;
  } else {
    return true;
  }
}


void UavStateMachine::ComputeOrientation(void) {
  if (failSafeMode) {
    GetDefaultOrientation()->GetQuaternionAndAngularRates(currentQuaternion,
                                                          currentAngularSpeed);
  } else {
    GetOrientation()->GetQuaternionAndAngularRates(currentQuaternion,
                                                   currentAngularSpeed);
  }
}

const AhrsData *UavStateMachine::GetOrientation(void) const {
  return GetDefaultOrientation();
}

const AhrsData *UavStateMachine::GetDefaultOrientation(void) const {
  return uav->GetAhrs()->GetDatas();
}

void UavStateMachine::ComputeAltitude(void) {
  if (failSafeMode) {
    FailSafeAltitudeValues(currentAltitude, currentVerticalSpeed);
  } else {
    AltitudeValues(currentAltitude, currentVerticalSpeed);
  }
}

void UavStateMachine::ComputeReferenceAltitude(float &refAltitude,
                                               float &refVerticalVelocity) {
  if (altitudeMode == AltitudeMode_t::Manual) {
    GetDefaultReferenceAltitude(refAltitude, refVerticalVelocity);
  } else {
    GetReferenceAltitude(refAltitude, refVerticalVelocity);
  }
}

void UavStateMachine::GetDefaultReferenceAltitude(float &refAltitude,
                                                  float &refVerticalVelocity) {
  float zc, dzc;

  switch (altitudeState) {
  // initiate a takeoff: increase motor speed in open loop (see ComputeThrust)
  // until we detect a take off of 0.03m (hard coded value) above the ground.
  case AltitudeState_t::TakingOff: {
    if (currentAltitude > groundAltitude + 0.03) {
      altitudeTrajectory->StartTraj(currentAltitude,
                                    desiredTakeoffAltitude->Value(),currentVerticalSpeed);
      altitudeState = AltitudeState_t::Stabilized;
      SignalEvent(Event_t::Stabilized);
    }
    break;
  }
  // landing, only check if we reach desired landing altitude
  //no break in this case, continue with Stabilized state
  case AltitudeState_t::StartLanding: {
    if (altitudeTrajectory->Position() == desiredLandingAltitude->Value()) {
      // The Uav target altitude has reached its landing value (typically 0)
      // but the real Uav altitude may not have reach this value yet because of
      // command delay. Moreover, it may never exactly reach this value if the
      // ground is not perfectly leveled (critical case: there's a
      // deep and narrow hole right in the sensor line of sight). That's why we
      // have a 2 phases landing strategy.
      altitudeState = AltitudeState_t::FinishLanding;
      SignalEvent(Event_t::FinishLanding);
      joy->SetLedOFF(1); // DualShock3::led1
    }
  }
  // stabilized: check if z trajectory is finished
  case AltitudeState_t::Stabilized: {
    if (!altitudeTrajectory->IsRunning() && !flagZTrajectoryFinished) {
      flagZTrajectoryFinished = true;
      SignalEvent(Event_t::ZTrajectoryFinished);
    }
    if (flagZTrajectoryFinished && desiredTakeoffAltitude->ValueChanged()) {
      flagZTrajectoryFinished = false;
      altitudeTrajectory->StartTraj(currentAltitude,
                                    desiredTakeoffAltitude->Value());
      joy->SetZRef(0);
    }
    break;
  }
  }

  // Récupère les consignes (du joystick dans l'implémentation par défaut). La
  // consigne joystick est une vitesse ("delta_z", dzc). le zc est calculé par
  // la manette
  zc = joy->ZRef(); // a revoir, la position offset devrait se calculer dans le
                    // generator
  dzc = joy->DzRef();

  // z control law
  altitudeTrajectory->SetPositionOffset(zc);
  altitudeTrajectory->SetSpeedOffset(dzc);

  altitudeTrajectory->Update(GetTime());
  refAltitude = altitudeTrajectory->Position();
  refVerticalVelocity = altitudeTrajectory->Speed();
}

void UavStateMachine::GetReferenceAltitude(float &refAltitude,
                                           float &refVerticalVelocity) {
  Thread::Warn("Default GetReferenceAltitude method is not overloaded => "
               "switching back to safe mode\n");
  EnterFailSafeMode();
  GetDefaultReferenceAltitude(refAltitude,refVerticalVelocity);
};

void UavStateMachine::ComputeThrust(void) {
  if (thrustMode == ThrustMode_t::Default) {
    currentThrust = ComputeDefaultThrust();
  } else {
    currentThrust = ComputeCustomThrust();
  }
}

float UavStateMachine::ComputeDefaultThrust(void) {
  if (needToComputeDefaultThrust) {
    // compute desired altitude
    float refAltitude, refVerticalVelocity;
    ComputeReferenceAltitude(refAltitude, refVerticalVelocity);

    switch (altitudeState) {
    case AltitudeState_t::TakingOff: {
      // The progressive increase in motor speed is used to evaluate the motor
      // speed that compensate the uav weight. This value
      // will be used as an offset for altitude control afterwards
      uZ->OffsetStepUp();
      break;
    }
    case AltitudeState_t::StartLanding:
    case AltitudeState_t::Stabilized: {
      float p_error = currentAltitude - refAltitude;
      float d_error = currentVerticalSpeed - refVerticalVelocity;
      uZ->SetValues(p_error, d_error);
      break;
    }
    // decrease motor speed in open loop until value offset_g , uav should have
    // already landed or be very close to at this point
    case AltitudeState_t::FinishLanding: {
      if (uZ->OffsetStepDown() == false) {
        StopMotors();
      }
      break;
    }
    }
    uZ->Update(GetTime());

    savedDefaultThrust = uZ->Output();
    needToComputeDefaultThrust = false;
  }

  return savedDefaultThrust;
}

float UavStateMachine::ComputeCustomThrust(void) {
  Thread::Warn("Default ComputeCustomThrust method is not overloaded => switching back "
               "to safe mode\n");
  EnterFailSafeMode();
  return ComputeDefaultThrust();
}

const AhrsData *UavStateMachine::ComputeReferenceOrientation(void) {
  if (orientationMode == OrientationMode_t::Manual) {
    return GetDefaultReferenceOrientation();
  } else {
    return GetReferenceOrientation();
  }
}

const AhrsData *UavStateMachine::GetDefaultReferenceOrientation(void) const {
  // We directly control yaw, pitch, roll angles
  return joy->GetReferenceOrientation();
}

const AhrsData *UavStateMachine::GetReferenceOrientation(void) {
  Thread::Warn("Default GetReferenceOrientation method is not overloaded => "
               "switching back to safe mode\n");
  EnterFailSafeMode();
  return GetDefaultReferenceOrientation();
}

void UavStateMachine::ComputeTorques(void) {
  if (torqueMode == TorqueMode_t::Default) {
    ComputeDefaultTorques(currentTorques);
  } else {
    ComputeCustomTorques(currentTorques);
  }
}

void UavStateMachine::ComputeDefaultTorques(Euler &torques) {
  if (needToComputeDefaultTorques) {
    const AhrsData *refOrientation = ComputeReferenceOrientation();
    Quaternion refQuaternion;
    Vector3Df refAngularRates;
    refOrientation->GetQuaternionAndAngularRates(refQuaternion,
                                                 refAngularRates);
    Euler refAngles = refQuaternion.ToEuler();
    Euler currentAngles = currentQuaternion.ToEuler();

    uYaw->SetValues(refAngles.yaw,
                    currentAngularSpeed.z - refAngularRates.z);
    uYaw->Update(GetTime());
    torques.yaw = uYaw->Output();

    uPitch->SetValues(refAngles.pitch, currentAngles.pitch,
                      currentAngularSpeed.y);
    uPitch->Update(GetTime());
    torques.pitch = uPitch->Output();

    uRoll->SetValues(refAngles.roll, currentAngles.roll, currentAngularSpeed.x);
    uRoll->Update(GetTime());
    torques.roll = uRoll->Output();

    savedDefaultTorques = torques;
    needToComputeDefaultTorques = false;
  } else {
    torques = savedDefaultTorques;
  }
}

void UavStateMachine::ComputeCustomTorques(Euler &torques) {
  Thread::Warn("Default ComputeCustomTorques method is not overloaded => "
               "switching back to safe mode\n");
  EnterFailSafeMode();
  ComputeDefaultTorques(torques);
}

void UavStateMachine::TakeOff(void) {
  flagZTrajectoryFinished = false;
  
  if(altitudeState!=AltitudeState_t::Stopped) {
    Warn("cannot takeoff, altitudeState!=AltitudeState_t::Stopped\n");
    joy->ErrorNotify();
  } else if(!safeToFly) {
    Warn("cannot takeoff, uav is not safe to fly\n");
    joy->ErrorNotify();
  } else if(!uav->isReadyToFly()) {
    Warn("cannot takeoff, uav is not ready\n");
    joy->ErrorNotify();
  } else if(uav->GetBatteryMonitor()->IsBatteryLow()) {
    Warn("cannot takeoff, battery is low\n");
    joy->ErrorNotify();
  } else if(flagConnectionLost) {
    Warn("cannot takeoff, connection with flairgcs lost\n");
    joy->ErrorNotify();
  } else {
    //The uav always takes off in fail safe mode
    flagBatteryLow=false;
    EnterFailSafeMode();
    joy->SetLedOFF(4);//DualShock3::led4
    joy->SetLedOFF(1);//DualShock3::led1
    joy->Rumble(0x70);
    joy->SetZRef(0);

    uZ->Reset();
    uRoll->Reset();
    uPitch->Reset();
    uYaw->Reset();

    uav->GetUavMultiplex()->LockUserInterface();
    // Active les moteurs. Pouvoir les désactiver permet de pouvoir observer les
    // consignes moteurs
    // sans les faire tourner effectivement (en déplaçant à la main le drone)
    uav->GetBldc()->SetEnabled(true);
    altitudeState = AltitudeState_t::TakingOff;
    SignalEvent(Event_t::TakingOff);
    ComputeAltitude();//ensure we have good altitude
    groundAltitude = currentAltitude;
  }
}

void UavStateMachine::Land(void) {
  if (thrustMode != ThrustMode_t::Default) {
    SetThrustMode(ThrustMode_t::Default);
  }
  if (altitudeMode != AltitudeMode_t::Manual) {
    SetAltitudeMode(AltitudeMode_t::Manual);
  }
  if (altitudeState == AltitudeState_t::Stabilized) {
    joy->SetLedOFF(4); // DualShock3::led4
    joy->Rumble(0x70);

    altitudeTrajectory->StopTraj();
    joy->SetZRef(0);
    ComputeAltitude();//ensure we have good altitude (Land() is not syncronized with Run())
    altitudeTrajectory->StartTraj(currentAltitude,desiredLandingAltitude->Value()); //shouldn't it be groundAltitude?
    altitudeState=AltitudeState_t::StartLanding;
    SignalEvent(Event_t::StartLanding);
  } else if (altitudeState==AltitudeState_t::TakingOff) {
    EmergencyLand();
  } else {
    joy->ErrorNotify();
  }
}

void UavStateMachine::EmergencyLand(void) {
    //Gradually decrease motor speed
    //Called if landing is required during take off (motors are accelerating but Uav did not actually left the ground yet), or if critical sensors have been lost (attitude is lost)
    altitudeState=AltitudeState_t::FinishLanding;
    safeToFly=false;
    Warn("Emergency landing!\n");
    Warn("You will not be able to take off again\n");
}

void UavStateMachine::SignalEvent(Event_t event) {
  switch (event) {
  case Event_t::StartLanding:
    Thread::Info("Altitude: entering 'StartLanding' state\n");
    break;
  case Event_t::Stopped:
    Thread::Info("Altitude: entering 'Stopped' state\n");
    break;
  case Event_t::TakingOff:
    Thread::Info("Altitude: taking off\n");
    break;
  case Event_t::Stabilized:
    Thread::Info("Altitude: entering 'Stabilized' state\n");
    break;
  case Event_t::FinishLanding:
    Thread::Info("Altitude: entering 'FinishLanding' state\n");
    break;
  case Event_t::EmergencyStop:
    Thread::Info("Emergency stop!\n");
    break;
  case Event_t::EnteringFailSafeMode:
    Thread::Info("Entering fail safe mode\n");
    break;
  }
}

void UavStateMachine::EmergencyStop(void) {
    if(altitudeState!=AltitudeState_t::Stopped) {
        StopMotors();
        EnterFailSafeMode();
        joy->Rumble(0x70);
        SignalEvent(Event_t::EmergencyStop);
    }
    //safeToFly=false;
    //Warn("Emergency stop, UAV will not take off again until program is rerunned\n");
}

void UavStateMachine::StopMotors(void) {
  joy->FlashLed(1, 10, 10); // DualShock3::led1
  uav->GetBldc()->SetEnabled(false);
  uav->GetUavMultiplex()->UnlockUserInterface();
  altitudeState = AltitudeState_t::Stopped;
  SignalEvent(Event_t::Stopped);
  uav->GetAhrs()->UnlockUserInterface();

  uZ->Reset();
  uRoll->Reset();
  uPitch->Reset();
  uYaw->Reset();
}

GridLayout *UavStateMachine::GetButtonsLayout(void) const {
  return buttonslayout;
}

void UavStateMachine::SecurityCheck(void) {
  MandatorySecurityCheck();
  ExtraSecurityCheck();
}

void UavStateMachine::MandatorySecurityCheck(void) {
  if (getFrameworkManager()->ConnectionLost() && !flagConnectionLost) {
    flagConnectionLost = true;
    
    EnterFailSafeMode();
    if (altitudeState == AltitudeState_t::Stopped) {
      Thread::Warn("Connection lost\n");
      Thread::Warn("UAV won't take off\n");
    } else {
      Thread::Err("Connection lost\n");
      Land();
    }
  }
    if((altitudeState==AltitudeState_t::TakingOff || altitudeState==AltitudeState_t::Stabilized) && uav->GetBatteryMonitor()->IsBatteryLow() && !flagBatteryLow) {
        flagBatteryLow=true;
        Thread::Err("Low Battery\n");
				LowBatteryAction();
    }/*
    Time now=GetTime();
    if ((altitudeState==AltitudeState_t::Stopped) && (now-uav->GetAhrs()->lastUpdate>(Time)100*1000*1000)) { //100ms
        flagCriticalSensorLost=true;
        Thread::Err("Critical sensor lost\n");
        EnterFailSafeMode();
        EmergencyLand();
    }*/
}

void UavStateMachine::LowBatteryAction(void) {
  Thread::Warn("LowBatteryAction method is not overloaded => switching back to default one\n");
  LowBatteryDefaultAction();
}

void UavStateMachine::LowBatteryDefaultAction(void) {
  EnterFailSafeMode();
	Land();
}

void UavStateMachine::CheckJoystick(void) {
  GenericCheckJoystick();
  ExtraCheckJoystick();
}

void UavStateMachine::GenericCheckJoystick(void) {
  static bool isEmergencyStopButtonPressed = false;
  static bool isTakeOffButtonPressed = false;
  static bool isSafeModeButtonPressed = false;

  if (controller->IsButtonPressed(1)) { // select
    if (!isEmergencyStopButtonPressed) {
      isEmergencyStopButtonPressed = true;
      Thread::Info("Emergency stop from joystick\n");
      EmergencyStop();
    }
  } else
    isEmergencyStopButtonPressed = false;

  if (controller->IsButtonPressed(0)) { // start
    if (!isTakeOffButtonPressed) {
      isTakeOffButtonPressed = true;
      switch (altitudeState) {
      case AltitudeState_t::Stopped:
        TakeOff();
        break;
      case AltitudeState_t::Stabilized:
        Land();
        break;
      default:
        joy->ErrorNotify();
        break;
      }
    }
  } else
    isTakeOffButtonPressed = false;

  // cross
  // gsanahuj:conflict with Majd programs.
  // check if l1,l2,r1 and r2 are not pressed
  // to allow a combination in user program
  if (controller->IsButtonPressed(5) && !controller->IsButtonPressed(6) &&
      !controller->IsButtonPressed(7) && !controller->IsButtonPressed(9) &&
      !controller->IsButtonPressed(10)) {
    if (!isSafeModeButtonPressed) {
      isSafeModeButtonPressed = true;
      EnterFailSafeMode();
    }
  } else
    isSafeModeButtonPressed = false;
}

void UavStateMachine::CheckPushButton(void) {
  GenericCheckPushButton();
  ExtraCheckPushButton();
}

void UavStateMachine::GenericCheckPushButton(void) {
  if (button_quit->Clicked() == true)
    SafeStop();
  if (button_take_off->Clicked() == true)
    TakeOff();
  if (button_land->Clicked() == true)
    Land();
  if (button_start_log->Clicked() == true) {
    SignalEvent(Event_t::StartLog);
    getFrameworkManager()->StartLog();
  }
  if (button_stop_log->Clicked() == true) {
    SignalEvent(Event_t::StopLog);
    getFrameworkManager()->StopLog();
  }
}

void UavStateMachine::EnterFailSafeMode(void) {
  if(altitudeState!=AltitudeState_t::StartLanding) SetAltitudeMode(AltitudeMode_t::Manual);//
  SetOrientationMode(OrientationMode_t::Manual);
  SetThrustMode(ThrustMode_t::Default);
  SetTorqueMode(TorqueMode_t::Default);

  GetDefaultOrientation()->GetQuaternionAndAngularRates(currentQuaternion,
                                                        currentAngularSpeed);
  joy->SetYawRef(currentQuaternion);
  uYaw->Reset();
  uPitch->Reset();
  uRoll->Reset();

  failSafeMode = true;
  SignalEvent(Event_t::EnteringFailSafeMode);
}

bool UavStateMachine::ExitFailSafeMode(void) {
  // only exit fail safe mode if in Stabilized altitude state
  // gsanahuj: pour la demo inaugurale on ne peut pas etre en failsafe
  // le ruban perturbe l'us
  /*
    if (altitudeState!=AltitudeState_t::Stabilized) {
        return false;
    } else*/ {
        Thread::Info("Leaving fail safe mode\n");
    failSafeMode = false;
    return true;
  }
}

bool UavStateMachine::SetTorqueMode(TorqueMode_t const &newTorqueMode) {
  if ((newTorqueMode == TorqueMode_t::Custom) && (failSafeMode)) {
    if (!ExitFailSafeMode())
      return false;
  }
  // When transitionning from Custom to Default torque mode, we should reset the
  // default control laws
  if ((torqueMode == TorqueMode_t::Custom) &&
      (newTorqueMode == TorqueMode_t::Default)) {
    uYaw->Reset();
    uPitch->Reset();
    uRoll->Reset();
  }
  torqueMode = newTorqueMode;
  return true;
}

bool UavStateMachine::SetAltitudeMode(AltitudeMode_t const &newAltitudeMode) {
  if ((newAltitudeMode == AltitudeMode_t::Custom) && (failSafeMode)) {
    if (!ExitFailSafeMode())
      return false;
  }
  altitudeMode = newAltitudeMode;
  //avoid starting trajectory at take off (it will be started when altitudeState==AltitudeState_t::Stabilized)
  if(altitudeState!=AltitudeState_t::Stopped) GotoAltitude(desiredTakeoffAltitude->Value());

  return true;
}

bool UavStateMachine::GotoAltitude(float desiredAltitude) {
  if (altitudeMode != AltitudeMode_t::Manual) {
    return false;
  }
  altitudeTrajectory->StartTraj(uav->GetMetaUsRangeFinder()->z(),
                                desiredAltitude);
  return true;
}

bool UavStateMachine::SetOrientationMode(
    OrientationMode_t const &newOrientationMode) {
  if ((newOrientationMode == OrientationMode_t::Custom) && (failSafeMode)) {
    if (!ExitFailSafeMode())
      return false;
  }
  // When transitionning from Custom to Manual mode we must reset to yaw
  // reference to the current absolute yaw angle,
  // overwise the Uav will abruptly change orientation
  if ((orientationMode == OrientationMode_t::Custom) &&
      (newOrientationMode == OrientationMode_t::Manual)) {
    joy->SetYawRef(currentQuaternion);
  }
  orientationMode = newOrientationMode;
  return true;
}

bool UavStateMachine::SetThrustMode(ThrustMode_t const &newThrustMode) {
  if ((newThrustMode == ThrustMode_t::Custom) && (failSafeMode)) {
    if (!ExitFailSafeMode())
      return false;
  }
  thrustMode = newThrustMode;
  return true;
}

NestedSat* UavStateMachine::GetURoll(void) { return uRoll;};
NestedSat* UavStateMachine::GetUPitch(void) { return uPitch;};
Pid* UavStateMachine::GetUYaw(void) { return uYaw;};
PidThrust* UavStateMachine::GetUZ(void) { return uZ;};
TrajectoryGenerator1D * UavStateMachine::GetAltitudeTrajectory(void) { return altitudeTrajectory;};
