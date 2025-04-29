//  created:    2020/12/21
//  filename:   UgvNavigation.h
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

#ifndef UgvNavigation_H
#define UgvNavigation_H

#include <Thread.h>

namespace flair {
    namespace core {
        class UdpSocket;
    }
    namespace gui {
        class PushButton;
        class DoubleSpinBox;
    }
    namespace filter {
        class TrajectoryGenerator2DCircle;
        class Pid;
    }
    namespace meta {
        class MetaVrpnObject;
    }
    namespace sensor {
        class TargetController;
    }
}

class UgvNavigation : public flair::core::Thread {
    public:
        UgvNavigation(std::string name,flair::sensor::TargetController *controller);
        ~UgvNavigation();

    private:

	enum class BehaviourMode_t {
            Manual,
            Circle,
            Receive,
            Default
        };

        void Run(void);
        void StartCircle(void);
        void StopCircle(void);
        void ComputeManualControls(void);
        void ComputeCircleControls(void);
        void SecurityCheck(void);
        void CheckMessages(void);
        void CheckJoystick(void);
        void CheckPushButton(void);
        void ComputeReceiveControls(void);
        void Defaultmode(void);

        flair::filter::Pid *uX, *uY;
        flair::gui::PushButton *startCircle,*stopCircle,*button_kill,*button_start_log,*button_stop_log, *stop;
        flair::gui::DoubleSpinBox *l;
        flair::gui::DoubleSpinBox *xCircleCenter,*yCircleCenter;
        flair::meta::MetaVrpnObject *ugvVrpn;
        flair::filter::TrajectoryGenerator2DCircle *circle;
        BehaviourMode_t behaviourMode;
        bool vrpnLost;
        flair::sensor::TargetController *controller;
        flair::core::UdpSocket *message;

        float acc, steering;
};

#endif // UgvNavigation_H
