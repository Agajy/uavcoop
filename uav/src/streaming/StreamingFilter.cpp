//  created:    2017/01/26
//  filename:   StreamingFilter.cpp
//
//  author:     Thomas Fuhrmann
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    StreamingFilter image flow over socket
//
//
/*********************************************************************/

#include "StreamingFilter.h"
#include <TcpSocket.h>
#include <FrameworkManager.h>
#include <Image.h>
#include <stdio.h>
#include <iostream>
#include <cstring>
#include <cmath>
// #include <VisionFilter.h>
// #include <Image.h>
// #include <typeinfo>
// #include <IODevice.h>

using std::string;
using namespace flair::core;

namespace flair {
namespace filter {

StreamingFilter::StreamingFilter(const core::IODevice *parent, std::string name, std::string ipAddress,
                               int ipPort, flair::meta::MetaVrpnObject * vrpnDrone, 
                               flair::meta::MetaVrpnObject *targetVrpn, flair::core::Quaternion* orientation) :  
IODevice(parent, name), suspendFlag(false) {

  // Socket stuff
  this->sendingSocket = new TcpSocket(getFrameworkManager(), "SndSkt", true, true);

  if (!sendingSocket->Connect(ipPort, ipAddress)) {
      Err("sendingSocket connect failed\n");
  }

  this->drone = vrpnDrone;
  this->target = targetVrpn;
  this->orientation = orientation;

  t0 = GetTime();
  t1 = GetTime();

}

StreamingFilter::~StreamingFilter(void) {}


void StreamingFilter::UpdateFrom(const io_data *data) {

  Image *img=(Image*)data;

  int bytesSend = 0;
  int imageSize = img->GetDataType().GetSize();
  int n_channels = 2;//2;//imageSize/76800;   2 for real camera, 3 for simulation
  imageSize = imageSize/n_channels;
//   StreamingData streamingData;
//   streamingData.img = img;
//   int streDataSize = imageSize + 2*sizeof(float);
  char streDataBuffer[153600];
  int j = 0;
  for (size_t i = 0; i < n_channels*imageSize; i+=n_channels)
  {
    streDataBuffer[j] = char((uint8_t(img->buffer[i])+2*uint8_t(img->buffer[i+n_channels-1]))/3);
    // streDataBuffer[j] = char((uint8_t(img->buffer[i])+uint8_t(img->buffer[i+n_channels-1])+uint8_t(img->buffer[i+n_channels-2]))/3.);
    j++;
  }

  // strncpy(streDataBuffer, img->buffer, imageSize);
//   memcpy(streDataBuffer, &streamingData, streDataSize);
  
  if (!suspendFlag) {
      data->GetMutex();

      Vector3Df uav_pos, ugv_pos, ugv_speed, uav_speed;//, uav_vel;
      this->drone->GetPosition(uav_pos);
      // this->drone->GetSpeed(uav_vel);
      this->target->GetPosition(ugv_pos);
    //   Vector3Df uav_speed, ugv_speed;
      this->target->GetSpeed(ugv_speed);
      this->drone->GetSpeed(uav_speed);

      // get UAV current angle
      Quaternion currQuat;
      this->drone->GetQuaternion(currQuat);
      float yaw_current = atan2(2.0 * (currQuat.q3 * currQuat.q0 + currQuat.q1 * currQuat.q2) , - 1.0 + 2.0 * (currQuat.q0 * currQuat.q0 + currQuat.q1 * currQuat.q1));
      // Euler currentAngless;
      // currQuat.ToEuler(currentAngless);
      // float yaw_current = currentAngless.yaw;

      float ugv_speed_mod = std::sqrt(std::pow(ugv_speed.x,2) + std::pow(ugv_speed.y,2));

      bytesSend = sendingSocket->SendMessage(streDataBuffer, imageSize, 0);
      // Warn("Only %d bytes...\n", bytesSend);
      if ((bytesSend) < 0) {
          Warn("Only %d bytes...\n", bytesSend);
      }
      
      // Warn("Only %d bytes...\n", bytesSend);
      // flair::core::Time t1 = GetTime();
      unsigned long long t2 = (GetTime()-t0)*1e-6;//ms  *1e-9
      float dt = (GetTime()-t1)*1e-9;
      t1 = GetTime();
      float uav_w = (yaw_current - yaw)/dt;
      yaw = yaw_current;
      
      if ((bytesSend = sendingSocket->SendMessage((char*)&t2, sizeof(unsigned long long), 0)) < 0) {
          Warn("Only %d bytes...\n", bytesSend);
      }

      if ((bytesSend = sendingSocket->SendMessage((char*)&ugv_pos.x, sizeof(float), 0)) < 0) {
          Warn("Only %d bytes...\n", bytesSend);
      }
      if ((bytesSend = sendingSocket->SendMessage((char*)&ugv_pos.y, sizeof(float), 0)) < 0) {
          Warn("Only %d bytes...\n", bytesSend);
      }
      if ((bytesSend = sendingSocket->SendMessage((char*)&uav_pos.x, sizeof(float), 0)) < 0) {
          Warn("Only %d bytes...\n", bytesSend);
      }
      if ((bytesSend = sendingSocket->SendMessage((char*)&uav_pos.y, sizeof(float), 0)) < 0) {
          Warn("Only %d bytes...\n", bytesSend);
      }
      if ((bytesSend = sendingSocket->SendMessage((char*)&uav_pos.z, sizeof(float), 0)) < 0) {
          Warn("Only %d bytes...\n", bytesSend);
      }
      if ((bytesSend = sendingSocket->SendMessage((char*)&uav_speed.x, sizeof(float), 0)) < 0) {
          Warn("Only %d bytes...\n", bytesSend);
      }
      if ((bytesSend = sendingSocket->SendMessage((char*)&uav_speed.y, sizeof(float), 0)) < 0) {
          Warn("Only %d bytes...\n", bytesSend);
      }
      if ((bytesSend = sendingSocket->SendMessage((char*)&uav_w, sizeof(float), 0)) < 0) {
          Warn("Only %d bytes...\n", bytesSend);
      }
      // if ((bytesSend = sendingSocket->SendMessage((char*)&uav_vel.z, sizeof(float), 0)) < 0) {
      //     Warn("Only %d bytes...\n", bytesSend);
      // }
    //   if ((bytesSend = sendingSocket->SendMessage((char*)&uav_pos.x, sizeof(float), 0)) < 0) {
    //       Warn("Only %d bytes...\n", bytesSend);
    //   }
    //   if ((bytesSend = sendingSocket->SendMessage((char*)&uav_pos.y, sizeof(float), 0)) < 0) {
    //       Warn("Only %d bytes...\n", bytesSend);
    //   }
    //   if ((bytesSend = sendingSocket->SendMessage((char*)&uav_pos.z, sizeof(float), 0)) < 0) {
    //       Warn("Only %d bytes...\n", bytesSend);
    //   }
    //   if ((bytesSend = sendingSocket->SendMessage((char*)&(this->orientation->q0), sizeof(float), 0)) < 0) {
    //       Warn("Only %d bytes...\n", bytesSend);
    //   }
    //   if ((bytesSend = sendingSocket->SendMessage((char*)&(this->orientation->q1), sizeof(float), 0)) < 0) {
    //       Warn("Only %d bytes...\n", bytesSend);
    //   }
    //   if ((bytesSend = sendingSocket->SendMessage((char*)&(this->orientation->q2), sizeof(float), 0)) < 0) {
    //       Warn("Only %d bytes...\n", bytesSend);
    //   }
    //   if ((bytesSend = sendingSocket->SendMessage((char*)&(this->orientation->q3), sizeof(float), 0)) < 0) {
    //       Warn("Only %d bytes...\n", bytesSend);
    //   }


      data->ReleaseMutex();
  }
  ProcessUpdate(img);
}

} // end namespace filter
} // end namespace flair
