// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//
//  filename:   ImageStreamServer.cpp
//
//  purpose:    Stream images over TCP socket to 127.0.0.1:50001 using Thread class
//
/*********************************************************************/

#include <TcpSocket.h>
#include <Thread.h>
#include <iostream>
#include <vector>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include "ImageStreamer.h"
#include <Camera.h>
#include <V4LCamera.h>
#include <CvtColor.h>
#include <arpa/inet.h>

using namespace std;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;
using namespace flair::sensor;

SimpleImage::SimpleImage(int width, int height) : width(width), height(height) {
    // Create a buffer for RGB data
    data.resize(width * height);
    
    // Initialize with a test pattern
    // regenerate(0);
}

// Update the image data for animation effects
void SimpleImage::regenerate(int frame,Image* myImage, int height, int width) {
    // Supposons que vous avez une image Image déjà créée
    // Et vous voulez copier son contenu dans un tableau data

    // D'abord, déterminez le format et les dimensions
    Image::Type::Format format = myImage->GetDataType().GetFormat();
    width = myImage->GetDataType().GetWidth();
    height = myImage->GetDataType().GetHeight();
    size_t pixelSize = myImage->GetDataType().GetPixelSize();
    size_t totalSize = width * height * pixelSize;

    myImage->GetMutex();

    memcpy(data.data(), myImage->buffer, totalSize);

    myImage->ReleaseMutex();

}
    
// Get raw image data pointer
const char* SimpleImage::getData() const {
    return reinterpret_cast<const char*>(data.data());
}
    
// Get size of image data in bytes
size_t SimpleImage::getDataSize() const {
    return data.size();
}

bool ImageStreamServer::initialize() {
    try {
        // Listen on the specified port
        serverSocket.Listen(62734, streamingIp);
        std::string message="Server listening on"+streamingIp+":62734";
        cout << message << endl;
        return true;
    } catch (const std::exception& e) {
        Err("Failed to initialize server: %s", e.what());
        return false;
    }
}

void ImageStreamServer::encodeHeader(ImageHeader& header) {
    header.magic = htonl(header.magic);
    header.width = htonl(header.width);
    header.height = htonl(header.height);
    header.channels = htonl(header.channels);
    header.frameNumber = htonl(header.frameNumber);
    header.timestamp = htonl(header.timestamp);
}

void ImageStreamServer::getImage(int frame){
    output=greyCameraImage->Output();
    int height=output->GetDataType().GetHeight();
    int width=output->GetDataType().GetWidth();

    image.regenerate(frame,output,height,width);
    
    return;
   
}

void ImageStreamServer::Run(void) {
    std::cout << __PRETTY_FUNCTION__ << " - line " << __LINE__ << ", " << ObjectName() << ": Waiting for client connection..." << std::endl;

    while (!ToBeStopped()) {
        if (clientSocket == nullptr) {
            clientSocket = serverSocket.Accept(0); // Timeout 100 ms
            if (clientSocket != nullptr) {
                std::cout << ObjectName() << ": Client connected!" << std::endl;
            }
        }

        if (clientSocket) {
            getImage(frameCounter);
            // if (frameCounter++>=1000){
            //     frameCounter=0;
            // }
            ImageHeader header = {
                .magic = 0x12345678,
                .width = (uint32_t)width,
                .height = (uint32_t)height,
                .channels = 1,
                .frameNumber = (uint32_t)frameCounter++,
                .timestamp = static_cast<uint32_t>(std::time(nullptr))
            };
            encodeHeader(header);
            
            clientSocket->SendMessage((char*)&header, sizeof(ImageHeader));

            // clientSocket->SendMessage((char*)&header, sizeof(ImageHeader));
            

            clientSocket->SendMessage(image.getData(), image.getDataSize());
        }
        WaitPeriod(); // respecter la période de 33ms (~30fps)
    }

    std::cout << ObjectName() << ": Thread stopped." << std::endl;
}