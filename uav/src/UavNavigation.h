//  created:    2011/05/01
//  filename:   UavNavigation.h
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

#ifndef UavNavigation_H
#define UavNavigation_H

#include <UavStateMachine.h>
#include "streaming/ServerTCP.h"
#include "Trajectory.cpp"

namespace flair {
    namespace core {
        class UdpSocket;
    }
    namespace gui {
        class PushButton;
    }
    namespace filter {
        class TrajectoryGenerator2DCircle;
        class SepSat; 
    }
    namespace meta {
        class MetaVrpnObject;
    }
    namespace sensor {
        class TargetController;
    }
}

class UavNavigation : public flair::meta::UavStateMachine {
    public:
        UavNavigation(flair::sensor::TargetController *controller, const std::string streaminIp);
        ~UavNavigation();

        // ServerTCP *server;
        float yaw;

    private:

	enum class BehaviourMode_t {
            Default,
            PositionHold,
            Circle
        };

        bool start_circle = true;
        bool isRunning = false;
        bool line_tracking_started = false;
        int index = 0;

        BehaviourMode_t behaviourMode;
        bool vrpnLost;

        void VrpnPositionHold(void);//flight mode
        void StartCircle(void);
        void StopCircle(void);
        void ExtraSecurityCheck(void);
        void ExtraCheckPushButton(void);
        void ExtraCheckJoystick(void);
        const flair::core::AhrsData *GetOrientation(void) const;
        void AltitudeValues(float &z,float &dz) const;
        void PositionValues(flair::core::Vector2Df &pos_error,flair::core::Vector2Df &vel_error, float &yaw_ref);
        flair::core::AhrsData *GetReferenceOrientation(void);
        void SignalEvent(Event_t event);

        // void GetReferenceAltitude(float &refAltitude,
        //                             float &refVerticalVelocity);

        // flair::filter::CvtColor* greyImageCameraVertical;

        flair::filter::Pid *uX, *uY;
        // flair::filter::SepSat *uX, *uY, *newuZ;
        flair::core::Time t0, t0_xy, t0_;
        float calculatedForceNorm;
        float altitude_error_integral, x_error_integral, y_error_integral;
        float desired_altitude;
        flair::core::Vector2Df pos_err, vel_err;
        bool subindo = false;

        flair::core::Vector2Df pos_error_prev;

        flair::core::Vector2Df posHold;
        float yawHold;

        flair::gui::PushButton *startCircle,*stopCircle,*positionHold;
        flair::meta::MetaVrpnObject *targetVrpn,*uavVrpn;
        flair::filter::TrajectoryGenerator2DCircle *circle;
        flair::core::AhrsData *customReferenceOrientation,*customOrientation;  

        flair::sensor::ServerTCP* server; 
        flair::core::Quaternion currDroneOrientation;     

        Trajectory trajectory;

        flair::core::UdpSocket *message;
};

#endif // UavNavigation_H
