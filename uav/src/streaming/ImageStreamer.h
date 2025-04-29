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
#include <Camera.h>
#include <V4LCamera.h>
#include <CvtColor.h>
#include <chrono>


namespace flair {
    namespace filter {
        class CvtColor;
    }
}

using namespace flair::core;
using namespace flair::filter;

// Simple image class to generate test images
class SimpleImage {
public:
    SimpleImage(int width, int height);
    
    // Update the image data for animation effects
    void regenerate(int frame,Image* myImage, int height, int width);
    
    
    // Get raw image data pointer
    const char* getData() const;
    
    // Get size of image data in bytes
    size_t getDataSize() const;
    
    // Get image dimensions
    int getWidth() const { return width; }
    int getHeight() const { return height; }
    
private:
    int width;
    int height;
    std::vector<uint8_t> data;
};

// Image header structure for network transmission
struct ImageHeader {
    uint32_t magic;      // Magic number for validation
    uint32_t width;      // Image width
    uint32_t height;     // Image height
    uint32_t channels;   // Number of color channels (3 for RGB)
    uint32_t frameNumber;// Frame counter
    uint32_t timestamp;  // Timestamp
};

// Server class to handle image streaming
class ImageStreamServer : public Thread {
public:
    ImageStreamServer(const Object* parent,flair::sensor::Camera *camera, const std::string& name, int width, int height, const std::string& Ip) 
        : Thread(parent, name, 50, 65536), // Medium priority, 64K stack
          serverSocket(this, "ServerSocket", true, true),
          width(width), height(height),
          frameCounter(0),
          image(width, height) {
        
        SetPeriodMS(100); 
        verticalCamera=camera;
        streamingIp=Ip;
        greyCameraImage=new CvtColor(verticalCamera,"gray",CvtColor::Conversion_t::ToGray);

        verticalCamera->UseDefaultPlot(greyCameraImage->Output());
    }
    
    ~ImageStreamServer() {
        // Make sure to stop the thread safely
        SafeStop();
        Join();
    }
    
    // Initialize the server
    bool initialize();
    void encodeHeader(ImageHeader &header);
    void getImage(int frame);
    flair::sensor::Camera *verticalCamera;

    

    
    
protected:
    // Thread run method - called when the thread starts
    void Run(void) override;
    
private:
    TcpSocket serverSocket;
    TcpSocket* clientSocket = nullptr;
    int width;
    int height;
    uint32_t frameCounter;
    SimpleImage image;
    Image* output;
    flair::filter::CvtColor* greyCameraImage;
    std::string streamingIp;
};
