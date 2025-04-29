//  created:    2017/02/02
//  filename:   Streaming.h
//
//  author:     Thomas Fuhrmann
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    streams data and receives computed information
//
//
/*********************************************************************/

#ifndef STREAMING_H
#define STREAMING_H

#define STREAMING
// Temporary define for the HIL
#define SIMU
#define REAL

#include <Thread.h>
#include <Quaternion.h>
#include <Vector3D.h>
#include <IODevice.h>
#include <VrpnClient.h>
#include <MetaVrpnObject.h>
// #include <CvtColor.h>
// #include "../imgProcessing/imgProcessing.h"
#include <chrono>
#include <sys/time.h>
// #include "receivedData.h"

struct tagData {
    // int nbDetect = 0;
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    float yaw = 0.0;
    // int id = 0;
    bool valid = false;
    // flair::core::Time time;
    float ugv_acc;
    float ugv_steering;
}__attribute__ ((packed));

namespace flair {
    namespace core {
        class TcpSocket;
        class Mutex;
        class Matrix;
        class OneAxisRotation;
    }
    namespace gui {
        class Label;
        class GroupBox;
    }
    namespace filter {
        class ImgProcessing;
        class StreamingFilter;
        class CvtColor;
    }
}

class Streaming : public flair::core::Thread, public flair::core::IODevice {

public:
    Streaming(const std::string streamIp, flair::meta::MetaVrpnObject * vrpnDrone, flair::meta::MetaVrpnObject *targetVrpn,
                flair::core::Quaternion* orientation);

    ~Streaming();

    tagData receivedData, last_receivedData;
    
protected:

private:
    // reimplement the run of the Thread class
    void Run();
    void UpdateFrom(const flair::core::io_data *data);

    flair::gui::Label *fps;
    flair::core::Matrix *outputMatrix;
    flair::core::Matrix *savingMatrix;
    flair::filter::CvtColor *greyCameraImage;
    // flair::filter::ImgProcessing *greyCameraImage;
    flair::filter::StreamingFilter *streamingCameraImage;
    flair::core::TcpSocket *receivingSocket;
    std::chrono::time_point <std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsedSeconds;

    bool recvDataFlag;
    bool isTagDetected;
    flair::core::Mutex *recvDataMutex;
    flair::core::Vector3Df recvPosition;
    int recvId;
    flair::core::Time recieveTimeDetection;
};

#endif // STREAMING_H
