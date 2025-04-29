//  created:    2017/02/02
//  filename:   Streaming.cpp
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

#include "Streaming.h"

#include <Uav.h>
#include <Camera.h>
// #include <CvtColor.h>
// #include "imgProcessing/CvtColor.h"
#include <GridLayout.h>
#include <Label.h>
#include <Tab.h>
#include <GroupBox.h>
#include <DataPlot1D.h>
#include <FrameworkManager.h>
#include <Matrix.h>
#include <Euler.h>
#include <Mutex.h>
#include <TcpSocket.h>
#include <OneAxisRotation.h>
#include <cmath>
#include <stdio.h>
#include <iostream>
// #include <VisionFilter.h>

#include "StreamingFilter.h"

using namespace std;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;
using namespace flair::meta;
using namespace flair::sensor;

Streaming::Streaming(const std::string streamIp, flair::meta::MetaVrpnObject * vrpnDrone, 
                    flair::meta::MetaVrpnObject *targetVrpn, flair::core::Quaternion* orientation)
    : Thread(getFrameworkManager(), "Streaming", 6)
    , IODevice(getFrameworkManager(), "StreamIO")
    , recvDataFlag(false) {

  // string dsp_file;


  // if(!InitVisionFilter("file="+dsp_file)) {
	// 		exit(1);
	// 	}

  Uav* uav = GetUav();
  if(uav->GetVerticalCamera() == NULL) {
    Printf("Exiting because no vertical camera has been found...\n");
    exit(1);
  }

  // Build GUI in a new Tab
  Tab* mainTab = new Tab(getFrameworkManager()->GetTabWidget(), "Streaming");
  fps = new Label(mainTab->NewRow(), "fps");

  // Init variables
  isTagDetected = false;

  // Descriptor for plotting data
  MatrixDescriptor* desc = new MatrixDescriptor(4, 1);
  desc->SetElementName(0, 0, "x");
  desc->SetElementName(1, 0, "y");
  desc->SetElementName(2, 0, "z");
  desc->SetElementName(3, 0, "id");
  outputMatrix = new Matrix((IODevice*)this, desc, floatType);
  delete desc;

  // Logs
  MatrixDescriptor* descLog = new MatrixDescriptor(4, 1);
  descLog->SetElementName(0, 0, "marqueur_x");
  descLog->SetElementName(1, 0, "marqueur_y");
  descLog->SetElementName(2, 0, "marqueur_z");
  descLog->SetElementName(3, 0, "marqueur_id");
  this->savingMatrix = new Matrix((IODevice*)this, descLog, floatType);
  delete descLog;
  AddDataToLog(this->savingMatrix);

  // Mutex to protect the access of the received data
  recvDataMutex = new Mutex(static_cast<Thread*>(this), "recvDataMutex");
  // Create filters, first convert image in gray and then send it over socket
  // greyCameraImage = new CvtColor(uav->GetVerticalCamera(), "gray", CvtColor::Conversion_t::ToGray);
#ifdef REAL
  // streamingCameraImage = new StreamingFilter(greyCameraImage, "stream", streamIp, 5000, vrpnDrone, targetVrpn, orientation);
  streamingCameraImage = new StreamingFilter(uav->GetVerticalCamera(), "stream", streamIp, 5000, vrpnDrone, targetVrpn, orientation);
#endif
// Create the socket to send processed data
#ifdef SIMU
  receivingSocket = new TcpSocket(getFrameworkManager(), "RcvSkt", false, true);
  if(!receivingSocket->Connect(5001, streamIp)) {
    Thread::Err("receivingSocket connect failed\n");
  }
#endif

  this->last_receivedData.x = 0;
  this->last_receivedData.y = 0;
  this->last_receivedData.z = 0;
  this->last_receivedData.yaw = 0;
  this->last_receivedData.ugv_acc = 0;
  this->last_receivedData.ugv_steering = 0;
  // this->receivedData.nbDetect = 0;
  // this->receivedData.id = 0;
  this->last_receivedData.valid = false;
}

Streaming::~Streaming() {
}



void Streaming::Run() {
  if(getFrameworkManager()->ErrorOccured()) {
    SafeStop();
    Thread::Err("An error occurred, we don't launch the Run loop.\n");
  }
  long bytesReceived;
  // tagData receivedData;
  Time fpsNow, fpsPrev;
  int fpsCounter = 0;
  start = std::chrono::system_clock::now();
  fpsPrev = GetTime();
  Euler recvEuler;
  float prevYaw=0;
  while(!ToBeStopped()) {
    bytesReceived = receivingSocket->RecvMessage((char*)&(this->receivedData), sizeof(tagData));
    if(bytesReceived == -1) {
      Thread::Err("RecvMessage failed, received bytes = %d\n", bytesReceived);
    } 
    else {
      // end = std::chrono::system_clock::now();
      // elapsedSeconds = end - start;
      // Fps counter
      // todo more accurate fps counter ?
      fpsCounter++;
      if(fpsCounter == 10) {
        fpsNow = GetTime();
        fps->SetText("fps: %.1f", fpsCounter / ((float)(fpsNow - fpsPrev) / 1000000000.));
        fpsCounter = 0;
        fpsPrev = fpsNow;
      }
      recvDataMutex->GetMutex();
      recvDataFlag = true;
      // isTagDetected = receivedData.nbDetect > 0;
      if (this->receivedData.valid){
        this->last_receivedData.x = this->receivedData.x;
        this->last_receivedData.y = this->receivedData.y;
        this->last_receivedData.z = this->receivedData.z;
        this->last_receivedData.yaw = this->receivedData.yaw;
        this->last_receivedData.valid = this->receivedData.valid;
        this->last_receivedData.ugv_acc = this->receivedData.ugv_acc;
        this->last_receivedData.ugv_steering = this->receivedData.ugv_steering;
      }
      recvPosition = Vector3Df(this->receivedData.x, this->receivedData.y, this->receivedData.z);
      // recvId = receivedData.id;

      // recieveTimeDetection = this->receivedData.time;
      recvDataMutex->ReleaseMutex();

      // Update the data from the camera frame to the uav frame
      outputMatrix->GetMutex();
      outputMatrix->SetValueNoMutex(0, 0, recvPosition.x);
      outputMatrix->SetValueNoMutex(1, 0, recvPosition.y);
      outputMatrix->SetValueNoMutex(2, 0, recvPosition.z);
      // outputMatrix->SetValueNoMutex(3, 0, recvId);
      outputMatrix->ReleaseMutex();

      outputMatrix->SetDataTime(GetTime());

      // Logs
      savingMatrix->GetMutex();
      savingMatrix->SetValueNoMutex(0, 0, recvPosition.x);
      savingMatrix->SetValueNoMutex(1, 0, recvPosition.y);
      savingMatrix->SetValueNoMutex(2, 0, recvPosition.z);
      // savingMatrix->SetValueNoMutex(3, 0, recvId);
      savingMatrix->ReleaseMutex();
      // savingMatrix->SetDataTime(this->receivedData.time);
      

      // start = std::chrono::system_clock::now();
      // if(isTagDetected) {
      //   outputMatrix->SetDataTime(receivedData.time);
      //   ProcessUpdate(outputMatrix);
      // }
    }
  }
}


// nothing because Streaming has no IO parent
void Streaming::UpdateFrom(const io_data* data) {}