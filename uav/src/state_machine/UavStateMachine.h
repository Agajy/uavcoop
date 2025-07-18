// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2014/04/29
//  filename:   UavStateMachine.h
//
//  author:     Gildas Bayard, Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    state machine for UAV
//
//
/*********************************************************************/

#ifndef UAVSTATEMACHINE_H
#define UAVSTATEMACHINE_H

#include <Thread.h>
#include <Vector2D.h>
#include <Vector3D.h>
#include <Euler.h>
#include <Quaternion.h>

namespace flair {
  namespace core {
    class FrameworkManager;
    class AhrsData;
    class io_data;
  }
  namespace gui {
    class PushButton;
    class GridLayout;
    class Tab;
    class DoubleSpinBox;
  }
  namespace filter {
    class ControlLaw;
    class NestedSat;
    class Pid;
    class PidThrust;
    class TrajectoryGenerator1D;
  }
  namespace sensor {
    class TargetController;
    class AltitudeSensor;
  }
  namespace meta {
    class MetaDualShock3;
    class Uav;
  }
}

namespace flair {
namespace meta {

/*! \class UavStateMachine
*
* \brief State machine for UAV
*  The Thread is created with 
*  the FrameworkManager as parent. FrameworkManager must be created before.
* The Thread is synchronized with Ahrs, unless a period is set with SetPeriodUS or
* SetPeriodMS
*/

class UavStateMachine : public core::Thread {
protected:
  enum class AltitudeMode_t { Manual, Custom };
  const AltitudeMode_t &GetAltitudeMode(void) const { return altitudeMode; }
  bool SetAltitudeMode(const AltitudeMode_t &altitudeMode);

  // uses TrajectoryGenerator1D *altitudeTrajectory to go to desiredAltitude
  // available in mode AltitudeMode_t::Manual
  // return true if goto is possible
  bool GotoAltitude(float desiredAltitude);

  enum class OrientationMode_t { Manual, Custom };
  const OrientationMode_t &GetOrientationMode(void) const {
    return orientationMode;
  }
  bool SetOrientationMode(const OrientationMode_t &orientationMode);

  enum class ThrustMode_t { Default, Custom };
  const ThrustMode_t &GetThrustMode() const { return thrustMode; }
  bool SetThrustMode(const ThrustMode_t &thrustMode);

  enum class TorqueMode_t { Default, Custom };
  const TorqueMode_t &GetTorqueMode(void) const { return torqueMode; }
  bool SetTorqueMode(const TorqueMode_t &torqueMode);

  enum class Event_t {
    EnteringFailSafeMode,
    EnteringControlLoop,
    StartLanding,
    FinishLanding,
    Stopped,
    TakingOff,
    EmergencyStop,
    Stabilized, // as soon as uav is 3cm far from the ground
    ZTrajectoryFinished,
    StartLog,
    StopLog,
  };

  UavStateMachine(sensor::TargetController* controller);
  ~UavStateMachine();

  const core::Quaternion &GetCurrentQuaternion(void) const;

  const core::Vector3Df &GetCurrentAngularSpeed(void) const;

  void Land(void);
  void EmergencyLand(void);
  void TakeOff(void);
  void EmergencyStop(void);
  //! Used to signal an event
  /*!
      \param event the event which occured
  */
  virtual void SignalEvent(Event_t event);

  virtual const core::AhrsData *GetOrientation(void) const;
  const core::AhrsData *GetDefaultOrientation(void) const;

  virtual void AltitudeValues(float &z, float &dz) const; // in uav coordinate!
  void EnterFailSafeMode(void);
  bool ExitFailSafeMode(void);
  void FailSafeAltitudeValues(float &z, float &dz) const; // in uav coordinate!
  
  /*!
  * \brief Set the FailSafe AltitudeSensor
  *
  * by default, the failsafe AltitudeSensor is the MetaUsRangeFinder
  * you can change it (by a vrpn one) using this method in the case us range finder is not working well
  * use it carefully, trusting only vrpn can be risquee
  *
  * \param altitudeSensor altitudeSensor to use when is failsafe
  */
  void SetFailSafeAltitudeSensor(sensor::AltitudeSensor *altitudeSensor);

  gui::GridLayout *GetButtonsLayout(void) const;
  virtual void ExtraSecurityCheck(void){};
  virtual void ExtraCheckJoystick(void){};
  virtual void ExtraCheckPushButton(void){};

  void GetDefaultReferenceAltitude(float &refAltitude,
                                   float &refVerticalVelocity);
  virtual void GetReferenceAltitude(float &refAltitude,
                                    float &refVerticalVelocity);
  // float GetDefaultThrustOffset(void);
  const core::AhrsData *GetDefaultReferenceOrientation(void) const;
  virtual const core::AhrsData *GetReferenceOrientation(void);

  /*!
  * \brief Compute Custom Torques
  *
  * Reimplement this method to use TorqueMode_t::Custom. \n
  * This method is called internally by UavStateMachine, do not call it by yourself. \n
  * See GetTorques if you need torques values.
  *
  * \param torques custom torques
  */
  virtual void ComputeCustomTorques(core::Euler &torques);

  /*!
  * \brief Compute Default Torques
  *
  * This method is called internally by UavStateMachine when using TorqueMode_t::Default. \n
  * Torques are only computed once by loop, other calls to this method will use previously computed torques.
  *
  * \param torques default torques
  */
  void ComputeDefaultTorques(core::Euler &torques);

  /*!
  * \brief Get Torques
  *
  * \return torques current torques
  */
  // const core::Euler &GetTorques() const;

  /*!
  * \brief Compute Custom Thrust
  *
  * Reimplement this method to use ThrustMode_t::Custom. \n
  * This method is called internally by UavStateMachine, do not call it by yourself. \n
  * See GetThrust if you need thrust value.
  *
  * \return custom Thrust
  */
  virtual float ComputeCustomThrust(void);

  /*!
  * \brief Compute Default Thrust
  *
  * This method is called internally by UavStateMachine when using ThrustMode_t::Default. \n
  * Thrust is only computed once by loop, other calls to this method will use previously computed thrust.
  *
  * \return default thrust
  */
  float ComputeDefaultThrust(void);

  /*!
  * \brief Get Thrust
  *
  * \return current thrust
  */
  // float GetThrust() const;

  /*!
  * \brief Add an IODevice to the control law logs
  *
  * The IODevice will be automatically logged among the Uz logs,
  * if logging is enabled (see IODevice::SetDataToLog, FrameworkManager::StartLog
  * and FrameworkManager::AddDeviceToLog). \n
  *
  * \param device IODevice to log
  */
  void AddDeviceToControlLawLog(const core::IODevice *device);

  /*!
  * \brief Add an io_data to the control law logs
  *
  * The io_data will be automatically logged among the Uz logs,
  * if logging is enabled (see IODevice::SetDataToLog, FrameworkManager::StartLog
  * and FrameworkManager::AddDeviceToLog). \n
  *
  * \param data io_data to log
  */
  void AddDataToControlLawLog(const core::io_data *data);
	
	/*!
  * \brief Low Battery Action
  *
  * Reimplement this method to have a custom action when low battery is detected \n
	* This action should land the drone!
  *
  */
  virtual void LowBatteryAction(void);
	
	/*!
  * \brief Low Battery Default Action
  *
  * Default action when low battery is detected is to land in failsafe mode
  *
  */
  void LowBatteryDefaultAction(void);

  sensor::TargetController *GetTargetController(void) const;
  MetaDualShock3 *GetJoystick(void) const;
  
  filter::NestedSat *GetURoll(void);
  filter::NestedSat *GetUPitch(void);
  filter::Pid *GetUYaw(void);
  filter::PidThrust *GetUZ(void);
  filter::TrajectoryGenerator1D *GetAltitudeTrajectory(void);

  gui::Tab *setupLawTab, *graphLawTab;

private:
  /*!
  \enum AltitudeState_t
  \brief States of the altitude state machine
  */
  enum class AltitudeState_t {
    Stopped,      /*!< the uav motors are stopped */
    TakingOff,    /*!< take off initiated. Motors accelerate progressively until
                     the UAV lift up */
    Stabilized,   /*!< the uav is actively maintaining its altitude */
    StartLanding, /*!< landing initiated. Altitude is required to reach the
                     landing altitude (0 by default) */
    FinishLanding /*!< motors are gradually stopped */
  };
  AltitudeState_t altitudeState;
  void ProcessAltitudeFiniteStateMachine();
  void ComputeReferenceAltitude(float &refAltitude, float &refVerticalVelocity);

  float groundAltitude; // effective altitude when the uav leaves the ground
  float currentAltitude, currentVerticalSpeed;

  bool failSafeMode;
  void SecurityCheck(void);
  void MandatorySecurityCheck(void);
  void CheckJoystick();
  void GenericCheckJoystick();
  void CheckPushButton(void);
  void GenericCheckPushButton(void);
  void Run(void) override;
  void StopMotors(void);
  bool IsValuePossible(float value,std::string desc);

  Uav *uav;
  MetaDualShock3 *joy;
  sensor::TargetController *controller;

  core::Quaternion currentQuaternion;
  core::Vector3Df currentAngularSpeed;

  const core::AhrsData *ComputeReferenceOrientation(void);

  void ComputeOrientation(void);
  void ComputeAltitude(void);

  void ComputeTorques(void);
  core::Euler currentTorques, savedDefaultTorques;
  bool needToComputeDefaultTorques;

  void ComputeThrust(void);
  float currentThrust, savedDefaultThrust;
  bool needToComputeDefaultThrust;

  gui::PushButton *button_quit, *button_take_off, *button_land,
      *button_start_log, *button_stop_log;
  gui::GridLayout *buttonslayout;
  gui::DoubleSpinBox *desiredTakeoffAltitude, *desiredLandingAltitude;
  AltitudeMode_t altitudeMode;
  OrientationMode_t orientationMode;
  ThrustMode_t thrustMode;
  TorqueMode_t torqueMode;
  bool flagBatteryLow;
  bool flagConnectionLost;
  bool flagCriticalSensorLost;
  bool flagZTrajectoryFinished;
  bool safeToFly;
  filter::NestedSat *uRoll, *uPitch;
  filter::Pid *uYaw;
  filter::PidThrust *uZ;
  filter::TrajectoryGenerator1D *altitudeTrajectory;
  sensor::AltitudeSensor *failSafeAltitudeSensor;
};
}; // end namespace meta
}; // end namespace flair
#endif // UAVSTATEMACHINE_H
