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

#ifndef CIRCLEFOLLOWER_H
#define CIRCLEFOLLOWER_H

#include "state_machine/UavStateMachine.h"
#include <DoubleSpinBox.h>
// #include <thread>
#include <Thread.h>
#include <Matrix.h>
#include "tcp_command/CommandServer.h"
#include "tcp_rec_command/tcp_rec_command_server.h"


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
        class CommandServer;
    }
}

class test_fleet : public flair::meta::UavStateMachine {
    public:
        test_fleet(flair::sensor::TargetController *controller, const std::string& name, const std::string& Ip, const std::string& Port);//,uint16_t listeningPort);//,std::string ugvName,uint16_t listeningPort);
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
            FollowPathUAV,
            UAVControlTCP,
        };
  
        BehaviourMode_t behaviourMode;
        bool vrpnLost;
        bool path_init;

        void VrpnPositionHold(void);//flight mode
        void ControledByTCP(void);
        void VrpnFollowPath(void);
        void StartCircle(void);
        void StopCircle(void);
        void ExtraSecurityCheck(void) override;
        void ExtraCheckPushButton(void) override;
        void ExtraCheckJoystick(void) override;
        const flair::core::AhrsData *GetOrientation(void) const override;
        void AltitudeValues(float &z,float &dz) const override;
        void PositionValues(flair::core::Vector2Df &pos_error,flair::core::Vector2Df &vel_error,float &yaw_e);
        flair::core::AhrsData *GetReferenceOrientation(void) override;
        void SignalEvent(Event_t event) override;
        float ComputeCustomThrust(void);
        flair::core::Matrix *matrix_path;
        float thrust;
        bool running;  // Flag pour arrêter proprement le thread
        bool line_detected=false;
        bool no_opti = false;
        float time;
        float ex_tcp, ey_tcp, eyaw_tcp;
        flair::filter::Pid *uX, *uY;
        flair::filter::PidThrust *uZ_custom;
        flair::core::Vector2Df posHold;
        flair::core::Vector2Df posHold2;
        float yawHold;
        int path_length;
        std::vector<std::pair<double, double>> pathUAV;

        flair::gui::PushButton *startCircle,*stopCircle,*positionHold,*followPathUAV, *UAVControlTCP;
        flair::meta::MetaVrpnObject *targetVrpn,*uavVrpn;
        flair::filter::TrajectoryGenerator2DCircle *circle;
        flair::core::AhrsData *customReferenceOrientation,*customOrientation;
        flair::gui::DoubleSpinBox *v;
        flair::core::CommandServer *commandServerUAV;
        RecCommandServer* commandRecorder;
};

#endif // CIRCLEFOLLOWER_H
