//  created:    2011/05/01
//  filename:   test_fleet.h
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

#ifndef CIRCLEFOLLOWER_H
#define CIRCLEFOLLOWER_H

#include <UavStateMachine.h>
#include <DoubleSpinBox.h>
// #include <thread>
#include <Thread.h>
#include <Matrix.h>
// #include "TimerThread.h"



namespace flair {
    namespace gui {
        class PushButton;
        class DoubleSpinBox;
    }
    namespace filter {
        class TrajectoryGenerator2DCircle;
    }
    namespace meta {
        class MetaVrpnObject;
    }
    namespace sensor {
        class TargetController;
    }
    namespace core {
        class Thread;
    }
}

class test_fleet : public flair::meta::UavStateMachine {
    public:
        test_fleet(flair::sensor::TargetController *controller);//,uint16_t listeningPort);//,std::string ugvName,uint16_t listeningPort);
        ~test_fleet();

        bool flag_followpath=false;
        bool Get_flag_followpath(void);
        void newpoint(void);
        int path_indice;
        void timerrun(void);

    private:

        enum class BehaviourMode_t {
            Default,
            PositionHold,
            Circle,
            // FollowUGV,
            // StopFollowUGV,
            FollowPathUAV,
        };
        // BehaviourMode_t GetBehaviourMode(void);

        BehaviourMode_t behaviourMode;
        bool vrpnLost;
        bool path_init;

        void VrpnPositionHold(void);//flight mode
        // void VrpnFollowUGV(void);
        // void VrpnStopFollowUGV(void);
        void VrpnFollowPath(void);
        void StartCircle(void);
        void StopCircle(void);
        void ExtraSecurityCheck(void) override;
        void ExtraCheckPushButton(void) override;
        void ExtraCheckJoystick(void) override;
        const flair::core::AhrsData *GetOrientation(void) const override;
        void AltitudeValues(float &z,float &dz) const override;
        void PositionValues(flair::core::Vector2Df &pos_error,flair::core::Vector2Df &vel_error,float &yaw_ref);
        flair::core::AhrsData *GetReferenceOrientation(void) override;
        void SignalEvent(Event_t event) override;
        float ComputeCustomThrust(void);
        flair::core::Matrix *matrix_path;
        float thrust;
        // std::thread timerThread;  // Ajout du thread comme membre
        // TimerThread* timerThread;
        bool running;  // Flag pour arrêter proprement le thread

        float time;

        flair::filter::Pid *uX, *uY;
        flair::filter::PidThrust *uZ_custom;
        flair::core::Vector2Df posHold;
        flair::core::Vector2Df posHold2;
        float yawHold;
        int path_length;
        std::vector<std::pair<double, double>> pathUAV;

        flair::gui::PushButton *startCircle,*stopCircle,*positionHold,*followPathUAV;//,*startFollowUGV, *stopFollowUGV, *followPathUAV;
        flair::meta::MetaVrpnObject *targetVrpn,*uavVrpn;
        flair::filter::TrajectoryGenerator2DCircle *circle;
        flair::core::AhrsData *customReferenceOrientation,*customOrientation;
        flair::gui::DoubleSpinBox *v;
};

#endif // CIRCLEFOLLOWER_H
