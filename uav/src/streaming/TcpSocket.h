// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
/*!
 * \file TcpSocket.h
 * \brief Class defining a Tcp socket
 * \author Gildas Bayard, Copyright Heudiasyc UMR UTC/CNRS 7253
 * \date 2015/04/28
 * \version 4.0
 */

#ifndef TCPSOCKET_H
#define TCPSOCKET_H

#include <ConnectedSocket.h>

namespace flair {
namespace core {
/*! \class TcpSocket
*
* \brief Class encapsulating a TCP socket
*
*/
class TcpSocket : public ConnectedSocket {
public:
  TcpSocket(const Object *parent, const std::string name,
            bool blockOnSend = false, bool blockOnReceive = true);
  ~TcpSocket();
  void Listen(const unsigned int port, const std::string localAddress = "ANY");
  TcpSocket *Accept(
      Time timeout = TIME_INFINITE);
  bool Connect(const unsigned int distantPort, const std::string distantAddress,
               Time timeout = TIME_INFINITE) override;
  ssize_t SendMessage(const char *message, size_t message_len, Time timeout = TIME_INFINITE) override;
  ssize_t RecvMessage(char *buf, size_t buf_len, Time timeout = TIME_INFINITE) override;

  uint16_t NetworkToHost16(uint16_t data) override;
  uint16_t HostToNetwork16(uint16_t data) override;
  uint32_t NetworkToHost32(uint32_t data) override;
  uint32_t HostToNetwork32(uint32_t data) override;

private:
  int socket; // socket file descriptor
  bool blockOnSend;
  bool blockOnReceive;
  bool isConnected;
  bool isListening;
  unsigned int distantPort;
  std::string distantAddress;
  char *sendRingBuffer, *recvRingBuffer;
};

} // end namespace core
} // end namespace flair

#endif // TCPSOCKET_H
