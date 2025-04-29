//  created:    2011/05/01
//  filename:   main.cpp
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    exemple de code uav
//
//
/*********************************************************************/

#include "test_fleet.h"
#include "UavNavigation.h"
#include <UavFactory.h>
#include <FrameworkManager.h>
#include <stdio.h>
#include <tclap/CmdLine.h>
#include <TargetEthController.h>
#include <Uav.h>
#include <VisionFilter.h>
// #include "streaming/Streaming.h"
// #include <Camera.h>
#include "streaming/ImageStreamer.h"


using namespace TCLAP;
using namespace std;
using namespace flair::core;
using namespace flair::meta;
using namespace flair::sensor;

string dsp_file;
string type_uav;
string log_path;
int port;
int ds3port;
string xml_file;
string name_uav; //, name_ugv;
string address;
string streamingIp;
// uint16_t listeningPort;

void parseOptions(int argc, char** argv);


int main(int argc, char* argv[]) {
    parseOptions(argc,argv);

    if(!InitVisionFilter("file="+dsp_file)) {
        exit(1);
    }

    FrameworkManager *manager;
    manager= new FrameworkManager(name_uav);
    manager->SetupConnection(address,port);
    manager->SetupUserInterface(xml_file);
    manager->SetupLogger(log_path);

    Uav* drone=CreateUav(name_uav,type_uav);//,"use_camv=false use_camh=false");
    TargetEthController *controller=new TargetEthController("Dualshock3",ds3port);
    test_fleet* demo=new test_fleet(controller);//,listeningPort);//,name_ugv,listeningPort);


    // Création du serveur d'image
   

    // UavNavigation* demo=new UavNavigation(controller,streamingIp);
//     TimerThread controllerwithtimer(demo);

    // Lancement du TimerThread dans un thread dédié
//     std::thread test_fleet([&demo]() {
//         demo->timerrun();
//     });
    ImageStreamServer* imageServer;
    flair::sensor::Camera* camera=drone->GetVerticalCamera();
    imageServer = new ImageStreamServer(nullptr,camera, "ImageServer", camera->Width(), camera->Height(), streamingIp);

    if (camera == nullptr) {
        std::cerr << "verticalCamera is null!" << std::endl;
    } else {
        std::cerr << "verticalCamera retrieved successfully." << std::endl;
    }


    if (!imageServer->initialize()) {
        std::cerr << "Échec de l'initialisation du serveur d'image." << std::endl;
        return -1;
    }

    imageServer->Start();  // Lancement du thread

    std::cout << " le serveur tourne..." << std::endl;
    demo->Start();
    demo->Join();

    imageServer->Join();
    delete imageServer;
    delete manager;

    CloseVisionFilter();

}

void parseOptions(int argc, char** argv) {
	try {

        CmdLine cmd("Command description message", ' ', "0.1");

        ValueArg<string> uavNameArg("n", "name", "uav name, also used for vrpn", true, "x4", "string");
        cmd.add(uavNameArg);
    
        // ValueArg<string> ugvNameArg("g", "ugvname", "ugv name, also used for vrpn", true, "car", "string");
        // cmd.add(ugvNameArg);
    
        ValueArg<string> typeArgUAV("t","type","uav type: ardrone2, hds_x4, hds_x8, hds_xufo, x4_simu, x8_simu or x4_simux (with x the number of the simulated uav)",true,"hds_x4","string");
        cmd.add(typeArgUAV);

        ValueArg<string> xmlArg("x","xml","fichier xml",true,"./reglages.xml","string");
        cmd.add( xmlArg );

        ValueArg<string> logsArg("l","logs","repertoire des logs",true,"/media/ram","string");
        cmd.add( logsArg );

        ValueArg<int> portArg("p","port","port pour station sol",true,9000,"int");
        cmd.add( portArg );

        ValueArg<string> addressArg("a","address","adresse station sol",true,"127.0.0.1","string");
        cmd.add( addressArg );

        ValueArg<int> ds3portArg("d","ds3_port","port pour ds3",false,20000,"int");
        cmd.add( ds3portArg );

        ValueArg <string> streamingArg("s", "streaming", "IP address for streaming", true, "127.0.0.1", "string");
        cmd.add(streamingArg);

        ValueArg<string> dspArg("v","dsp","executable dsp",true,"./dspcv_dsp.out","string");
        cmd.add( dspArg );

        cmd.parse( argc, argv );

        // Get the value parsed by each arg.
        log_path = logsArg.getValue();
        port=portArg.getValue();
        ds3port=ds3portArg.getValue();
        xml_file = xmlArg.getValue();
        name_uav=uavNameArg.getValue();
        type_uav=typeArgUAV.getValue();
        address=addressArg.getValue();
        streamingIp = streamingArg.getValue();
        dsp_file = dspArg.getValue();

	} catch (ArgException &e) { // catch any exceptions
        cerr << "error: " << e.error() << " for arg " << e.argId() << endl;
        exit(EXIT_FAILURE);
	}
}
