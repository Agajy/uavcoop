// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2015/04/28
//  filename:   TcpSocket.cpp
//
//  author:     Gildas Bayard
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    Class defining a TCP socket
//
//
/*********************************************************************/
#include "TcpSocket.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <system_error>

using std::string;

namespace flair {
namespace core {

TcpSocket::TcpSocket(const Object *parent, const std::string name,
                     bool _blockOnSend, bool _blockOnReceive)
    : ConnectedSocket(parent, name), socket(0), isConnected(false), isListening(false), distantPort(0), distantAddress() {
  blockOnSend = _blockOnSend;
  blockOnReceive = _blockOnReceive;
}

TcpSocket::~TcpSocket() {
  // Info("Debug: destroying TCP socket %s", ObjectName().c_str());
  if (socket) close(socket);
}

void TcpSocket::Listen(const unsigned int port,
                       const std::string localAddress) {
  socket = ::socket(AF_INET, SOCK_STREAM, 0);

  sockaddr_in my_addr;
  my_addr.sin_family = AF_INET;
  my_addr.sin_port = htons(port);
  if (localAddress == "ANY") {
    my_addr.sin_addr.s_addr = INADDR_ANY;
  } else {
    inet_aton(localAddress.c_str(), &(my_addr.sin_addr));
  }
  memset(&(my_addr.sin_zero), '\0', 8);

  if (bind(socket, (sockaddr *)&my_addr, sizeof(my_addr)) < 0) {
    char errorMsg[256];
    Err("TCP bind, %s\n", strerror_r(errno, errorMsg, sizeof(errorMsg)));
  }

  int ret=listen(socket, 1);
  if (ret < 0) {
    char errorMsg[256];
    Err("select: %s\n", strerror_r(errno, errorMsg, sizeof(errorMsg)));
  } else {
    isListening=true;
  }
}

TcpSocket *TcpSocket::Accept(Time timeout) {
  if (!isListening) throw std::logic_error("Can't call Accept on a non listening socket");

  TcpSocket *acceptedSocket = nullptr;

  struct timeval tv;
  if (timeout != TIME_NONBLOCK) {
    tv.tv_sec = timeout / 1000000000;
    tv.tv_usec = (timeout % 1000000000) / 1000;
  } else {
    tv.tv_sec =0;
    tv.tv_usec =0;
  }
  fd_set rset;
  FD_ZERO(&rset);
  FD_SET(socket, &rset);

  int ret = select(socket + 1, &rset, nullptr, nullptr,
                   (timeout == TIME_INFINITE) ? nullptr : &tv); // man 2 accept
  if (ret < 0) {
    char errorMsg[256];
    Err("select: %s\n", strerror_r(errno, errorMsg, sizeof(errorMsg)));
  } else {
    if (ret == 0) {
      // timeout reached
      throw std::runtime_error("Timeout");
    } else {
      // our socket is readable, a new connection can be accepted
      acceptedSocket = new TcpSocket(this->Parent(), this->ObjectName(),
                                     blockOnSend, blockOnReceive);
      sockaddr_in their_addr;
      socklen_t namelen = sizeof(their_addr);
      ret = accept(socket, (sockaddr *)&their_addr, &namelen);
      if (ret < 0) {
        delete acceptedSocket;
        throw std::system_error(std::error_code(ret, std::generic_category()));
      } else {
        acceptedSocket->socket = ret;
      }
    }
  }

  return acceptedSocket;
}

bool TcpSocket::Connect(const unsigned int _distantPort,
                        const std::string _distantAddress, Time timeout) {
  bool success = false;
  bool criticalError=false; //meaning: if true then this socket can't be re-used

  //should we open a new socket, close it, or re-use it?
  if (isConnected) {
    // if already connected to the same host, just return true
    if (distantPort == _distantPort && distantAddress == _distantAddress) {
      return true;
    }
    // if already connected, but to a different host, close the current socket
    else {
      isConnected=false;
      close(socket);
      socket=0;
    }
  }
  if (!socket) {
    socket = ::socket(AF_INET, SOCK_STREAM, 0);
    if (socket == -1)
      return false;
  }

  sockaddr_in serv_addr;
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(short(_distantPort));
  if (inet_pton(AF_INET, _distantAddress.c_str(), &serv_addr.sin_addr) <= 0) {
    Warn("incorrect network address.\n");
    close(socket);
    socket=0;
    return false;
  }
  memset(&(serv_addr.sin_zero), '\0', 8);
  distantPort=_distantPort;
  distantAddress=_distantAddress;

  // go non blocking
  int flags = fcntl(socket, F_GETFL);
  fcntl(socket, F_SETFL, flags | O_NONBLOCK);
  if (connect(socket, (sockaddr *)&serv_addr, sizeof(serv_addr)) == -1) {
    if ((errno != EINPROGRESS) && (errno != EAGAIN)) {
      char errorMsg[256];
      Err("socket connect: %s\n",
          strerror_r(errno, errorMsg, sizeof(errorMsg)));
      criticalError = true;
    } else {
      // now block with a timeout
      struct timeval tv;
      if (timeout != 0) {// timeout is in ns
        tv.tv_sec = timeout /((Time)1000000000); 
        tv.tv_usec = (timeout%((Time)1000000000))/1000;
      }
      fd_set wset;
      FD_ZERO(&wset);
      FD_SET(socket, &wset);
      int ret =
          select(socket + 1, NULL, &wset, NULL,
                 (timeout == 0) ? NULL : &tv); // man 2 connect EINPROGRESS
      if (ret < 0) {
        char errorMsg[256];
        Err("select: %s\n", strerror_r(errno, errorMsg, sizeof(errorMsg)));
        criticalError = true;
      } else {
        if (ret != 0) { //if ret==0 we're on timeout (nothing to do since success is already set to false)
          // something happened on our socket. Check if an error occurred
          int error;
          socklen_t len = sizeof(error);
          if (getsockopt(socket, SOL_SOCKET, SO_ERROR, &error, &len) != 0) {
            //char errorMsg[256];
            // Err("getsockopt: %s\n",strerror_r(errno,errorMsg,sizeof(errorMsg)));
            criticalError = true;
          } else if (error != 0) {
            //char errorMsg[256];
            // Err("socket error: %d(%s)\n",error,strerror_r(error,errorMsg,sizeof(errorMsg)));
            criticalError = true;
          } else {
            if (connect(socket, (sockaddr *)&serv_addr, sizeof(serv_addr)) != -1) {
              // Info("connected indeed ^^\n");
              success = true;
            } else {
              //connect failed
              if ((errno != EINPROGRESS) && (errno != EAGAIN)) {
                criticalError = true;
              }
            }
          }
        }
      }
    }
  } else {
    success = true; // should never happen since we go non blocking and connect can't be immediate
  }
  // switch back to blocking mode (default)
  fcntl(socket, F_SETFL, flags);

  if (!success) {
    if (criticalError) {
      close(socket);
      socket=0;
    }
    isConnected=false;
     //Info("Debug: Connect to %s:%d failed\n", distantAddress.c_str(), distantPort);
    return false;
  } else {
    isConnected = true;
    distantPort = _distantPort;
    distantAddress = _distantAddress;
     //Info("Debug: Connect to %s:%d succeeded\n", distantAddress.c_str(), distantPort);
    return true;
  }
}

ssize_t TcpSocket::SendMessage(const char *buffer, size_t bufferSize,
                               Time timeout) {
  int flags = 0;
  if (!blockOnSend) {
    flags |= MSG_DONTWAIT;
  } else {
    struct timeval tv;
    tv.tv_sec = timeout / 1000000000; // timeout is in ns
    tv.tv_usec = (timeout % 1000000000) / 1000;

    setsockopt(socket, SOL_SOCKET, SO_SNDTIMEO, (char *)&tv,
               sizeof(struct timeval));
  }
  ssize_t bytesSent = send(socket, buffer, bufferSize, flags);

  return bytesSent;
}

ssize_t TcpSocket::RecvMessage(char *buffer, size_t bufferSize, Time timeout) {
  int flags = 0;
  if (!blockOnReceive) {
    flags |= MSG_DONTWAIT;
  } else {
    struct timeval tv;
    tv.tv_sec = timeout / 1000000000; // timeout is in ns
    tv.tv_usec = (timeout % 1000000000) / 1000;

    setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,
               sizeof(struct timeval));
  }
  ssize_t bytesRead = recv(socket, buffer, bufferSize, flags);

  return bytesRead;
}

uint16_t TcpSocket::NetworkToHost16(uint16_t data) { return ntohs(data); }

uint16_t TcpSocket::HostToNetwork16(uint16_t data) { return htons(data); }

uint32_t TcpSocket::NetworkToHost32(uint32_t data) { return ntohl(data); }

uint32_t TcpSocket::HostToNetwork32(uint32_t data) { return htonl(data); }

} // end namespace core
} // end namespace flair
