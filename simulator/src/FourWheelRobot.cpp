// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2020/11/20
//  filename:   FourWheelRobot.cpp
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    classe definissant un FourWheelRobot
//
/*********************************************************************/

//  CHANGE THIS FILE TO WORK WITH 4 WHEEL ROBOT
// you need to rewrite the code for the model (CalcModel method), and the drawing code (Draw method).

#include "FourWheelRobot.h"
#include <TabWidget.h>
#include <Tab.h>
#include <PushButton.h>
#include <GridLayout.h>
#include <DoubleSpinBox.h>
#include <SpinBox.h>
#include <GroupBox.h>
#include <math.h>
#include "MeshSceneNode.h"
#include <SimuUgvControls.h>
#ifdef GL
#include <ISceneManager.h>
#include <IMeshManipulator.h>
#include "MeshSceneNode.h"
#include "Gui.h"
#include <Mutex.h>
#include <iostream>

#endif

#define G (float)9.81 // gravity ( N/(m/s²) )

#ifdef GL
using namespace irr::video;
using namespace irr::scene;
using namespace irr::core;
#endif
using namespace flair::core;
using namespace flair::gui;
using namespace flair::actuator;

namespace flair {
namespace simulator {

FourWheelRobot::FourWheelRobot(std::string name, uint32_t modelId)
    : Model(name,modelId) {
  Tab *setup_tab = GetParamsTab();
  // Tab *setup_tab = new Tab(GetTabWidget(), "model");
  m = new DoubleSpinBox(setup_tab->NewRow(), "mass (kg):", 0, 20, 0.1,1,0.2);
  size = new DoubleSpinBox(setup_tab->NewRow(), "size (m):", 0, 20, 0.1,1,0.1);
  t_speed = new DoubleSpinBox(setup_tab->NewRow(), "translational speed (m/s):",
                              0, 5, 0.1);
  r_speed = new DoubleSpinBox(setup_tab->NewRow(), "rotational speed (deg/s):",
                              0, 180, 10);
  
  
  Tab *visual_tab = new Tab(GetTabWidget(), "visual");
  bodyColorR = new SpinBox(visual_tab->NewRow(), "arm color (R):", 0, 255, 1,255);
  bodyColorG = new SpinBox(visual_tab->LastRowLastCol(), "arm color (G):", 0, 255, 1,0);
  bodyColorB = new SpinBox(visual_tab->LastRowLastCol(), "arm color (B):", 0, 255, 1,0);
  
 
  
  controls = new SimuUgvControls(this, name, modelId,0);
  Tab *ugvTab = new Tab(GetTabWidget(), "model");

  GridLayout* buttonslayout = new GridLayout(ugvTab->NewRow(), "model robots");

  TwoWheelsButton = new PushButton(buttonslayout->NewRow(), "2 wheels robot");
  FourWheelsButton = new PushButton(buttonslayout->NewRow(), "4 wheels robot");  
  model ="TwoWheels";
  SetIsReady(true);
}

FourWheelRobot::~FourWheelRobot() {
  // les objets irrlicht seront automatiquement detruits par parenté
}


void FourWheelRobot::CheckPushButton(void) {
  if (TwoWheelsButton->Clicked() == true)
    model = "TwoWheels";
  if (FourWheelsButton->Clicked() == true)
    model = "FourWheels";
}

#ifdef GL

void FourWheelRobot::Draw(void) {
  // create unite (1m=100cm) robot; scale will be adapted according to settings in gcs
  // parameter
  // note that the frame used is irrlicht one:
  // left handed, North East Up

  // const IGeometryCreator *geo;
  // geo = getGui()->getSceneManager()->getGeometryCreator();

  // colored_body = geo->createCubeMesh(vector3df(ToIrrlichtScale(1),ToIrrlichtScale(1),ToIrrlichtScale(1)));
  // colored_body->setBoundingBox(aabbox3df(0,0,0,1,1,1));//bug with bounding box? workaround is to reduce it... we use only wheel box
  // MeshSceneNode* mesh= new MeshSceneNode(this, colored_body);
  
  // IMesh *wheel = geo->createCylinderMesh(ToIrrlichtScale(0.35), ToIrrlichtScale(0.1), 64, SColor(0, 0, 0, 0));
	// Vector3Df wheel_position(0,0.5,0.3);
  // MeshSceneNode *l_wheel = new MeshSceneNode(this, wheel, ToIrrlichtCoordinates(wheel_position),vector3df(0, 0, 0));
	// wheel_position=Vector3Df(0,-0.6,0.3);
  // MeshSceneNode *r_wheel = new MeshSceneNode(this, wheel, ToIrrlichtCoordinates(wheel_position),vector3df(0, 0, 0));

  const IGeometryCreator *geo;
  geo = getGui()->getSceneManager()->getGeometryCreator();

  colored_body = geo->createCubeMesh(vector3df(ToIrrlichtScale(1),ToIrrlichtScale(1),ToIrrlichtScale(1)));
  colored_body->setBoundingBox(aabbox3df(0,0,0,0.025,0.025,0.05));//bug with bounding box? workaround is to reduce it... we use only wheel box
  MeshSceneNode* mesh= new MeshSceneNode(this, colored_body);
  
  IMesh *wheel = geo->createCylinderMesh(30, 10, 64, SColor(0, 0, 0, 0));
  MeshSceneNode *rl_wheel = new MeshSceneNode(this, wheel, vector3df(-50, 50, -25),vector3df(0, 0, 0));
  MeshSceneNode *rr_wheel = new MeshSceneNode(this, wheel, vector3df(-50, -50-10, -25),vector3df(0, 0, 0));
  MeshSceneNode *fl_wheel = new MeshSceneNode(this, wheel, vector3df(50, 50, -25),vector3df(0, 0, 0));
  MeshSceneNode *fr_wheel = new MeshSceneNode(this, wheel, vector3df(50, -50-10, -25),vector3df(0, 0, 0));
  
 ExtraDraw();
}

void FourWheelRobot::AnimateModel(void) {
  if (bodyColorR->ValueChanged() == true || bodyColorG->ValueChanged() == true || bodyColorB->ValueChanged() == true) {
    getGui()->getSceneManager()->getMeshManipulator()->setVertexColors(colored_body, SColor(0,bodyColorR->Value(), bodyColorG->Value(), bodyColorB->Value()));
  }

  // adapt robot size
  if (size->ValueChanged() == true) {
    setScale(size->Value());
  }
   
}

size_t FourWheelRobot::dbtSize(void) const {
  return 3 * sizeof(float) + 2 * sizeof(float); // 3ddl+2motors
}

void FourWheelRobot::WritedbtBuf(
    char *dbtbuf) { /*
                       float *buf=(float*)dbtbuf;
                       vector3df vect=getPosition();
                       memcpy(buf,&vect.X,sizeof(float));
                       buf++;
                       memcpy(buf,&vect.Y,sizeof(float));
                       buf++;
                       memcpy(buf,&vect.Z,sizeof(float));
                       buf++;
                       vect=getRotation();
                       memcpy(buf,&vect.X,sizeof(float));
                       buf++;
                       memcpy(buf,&vect.Y,sizeof(float));
                       buf++;
                       memcpy(buf,&vect.Z,sizeof(float));
                       buf++;
                       memcpy(buf,&motors,sizeof(rtsimu_motors));*/
}

void FourWheelRobot::ReaddbtBuf(
    char *dbtbuf) { /*
                       float *buf=(float*)dbtbuf;
                       vector3df vect;
                       memcpy(&vect.X,buf,sizeof(float));
                       buf++;
                       memcpy(&vect.Y,buf,sizeof(float));
                       buf++;
                       memcpy(&vect.Z,buf,sizeof(float));
                       buf++;
                       setPosition(vect);
                       memcpy(&vect.X,buf,sizeof(float));
                       buf++;
                       memcpy(&vect.Y,buf,sizeof(float));
                       buf++;
                       memcpy(&vect.Z,buf,sizeof(float));
                       buf++;
                       ((ISceneNode*)(this))->setRotation(vect);
                       memcpy(&motors,buf,sizeof(rtsimu_motors));
                       AnimateModele();*/
}
#endif // GL

// states are computed on fixed frame NED
// x north
// y east
// z down

////////////////////////////////////////////////// Attention à remmetrre peut-etre si l'ugv est controllé en ACC
void FourWheelRobot::CalcModel(void) {
  // CheckPushButton();
  // if (model == "FourWheels"){
    // float acc,steering;
    // Time motorTime;

    // controls->GetControls(&acc,&steering,&motorTime);
    // steering = -steering;

    // float lr = 0.14;
    // float lf = 0.14;
    // float L = lf+lr;

    // // Update speeed
    // float ac_corr = acc;
    // if (ac_corr<-0.4) ac_corr = -0.4;
    // if (ac_corr>0.4) ac_corr = 0.4;
    // // ac_corr = ac_corr - std::pow(speed,2)*0.1;
    // // speed = speed*0.3 + 0.7*(speed + ac_corr*dT());
    // speed = speed + ac_corr*dT();
    // float vel_max = 0.2;
    // std::cout << "acccc bef: " << acc << std::endl;
    // std::cout << "speed bef: " << speed << std::endl;
    // if (speed>vel_max) speed = vel_max;
    // if (speed<-vel_max) speed = -vel_max;
    // // if (std::abs(ac_corr)>0)
    // //   speed = 0.2;

    // // update yaw rate
    // float steering_max = 0.65;
    // if (steering<-steering_max) steering = -steering_max;
    // if (steering>steering_max) steering = steering_max;
    // float beta = std::atan(lr*std::tan(steering)/L);
    // // state[-1].Vel.x
    // float turn = speed*std::cos(beta)*std::atan(steering)/L;

    
    // // compute quaternion from W
    // // Quaternion derivative: dQ = 0.5*(  Q*Qw)
    // state[0].W.x=0;
    // state[0].W.y=0;
    // state[0].W.z=turn;//*r_speed->Value();
    // Quaternion dQ = state[-1].Quat.GetDerivative(state[0].W);
    // // Quaternion dQ(turn, state[-1].Quat.q1, state[-1].Quat.q2, state[-1].Quat.q3);

    // // // Quaternion integration
    // state[0].Quat = state[-1].Quat + dQ * dT();
    // state[0].Quat.Normalize();

    // Vector3D<double> dir = Vector3D<double>(speed*t_speed->Value(),0,0);
    // dir.Rotate(state[0].Quat);
    // state[0].Pos = state[-1].Pos + dT() * dir;
  
    // /*
    // ** ===================================================================
    // **     z double integrator
    // **
    // ** ===================================================================
    // */
    // state[0].Pos.z = (dT() * dT() / m->Value()) * ( m->Value() * G) + 2 * state[-1].Pos.z - state[-2].Pos.z;
    // state[0].Vel.z = (state[0].Pos.z - state[-1].Pos.z) / dT();
  // }
  // if (model == "TwoWheels"){
    float speed,turn;
    Time motorTime;

    controls->GetControls(&speed,&turn,&motorTime);
    
    // compute quaternion from W
    // Quaternion derivative: dQ = 0.5*(  Q*Qw)
    state[0].W.x=0;
    state[0].W.y=0;
    state[0].W.z=turn*r_speed->Value();
    Quaternion dQ = state[-1].Quat.GetDerivative(state[0].W);

    // Quaternion integration
    state[0].Quat = state[-1].Quat + dQ * dT();
    state[0].Quat.Normalize();

    Vector3D<double> dir = Vector3D<double>(speed*t_speed->Value(),0,0);
    dir.Rotate(state[0].Quat);
    state[0].Pos = state[-1].Pos + dT() * dir;
    
  
    /*
    ** ===================================================================
    **     z double integrator
    **
    ** ===================================================================
    */
    state[0].Pos.z = (dT() * dT() / Mass()) * ( Mass() * G) + 2 * state[-1].Pos.z - state[-2].Pos.z;
    state[0].Vel.z = (state[0].Pos.z - state[-1].Pos.z) / dT();
  // }
}
//////////////////////////////////////////////////

// void FourWheelRobot::CalcModel(void) {
//   float speed,turn;
//   Time motorTime;

//   controls->GetControls(&speed,&turn,&motorTime);
	 
//   // compute quaternion from W
//   // Quaternion derivative: dQ = 0.5*(  Q*Qw)
//   state[0].W.x=0;
//   state[0].W.y=0;
//   state[0].W.z=turn*r_speed->Value();
//   Quaternion dQ = state[-1].Quat.GetDerivative(state[0].W);

//   // Quaternion integration
//   state[0].Quat = state[-1].Quat + dQ * dT();
//   state[0].Quat.Normalize();

//   Vector3D<double> dir = Vector3D<double>(speed*t_speed->Value(),0,0);
//   dir.Rotate(state[0].Quat);
//   state[0].Pos = state[-1].Pos + dT() * dir;
  
 
//   /*
//   ** ===================================================================
//   **     z double integrator
//   **
//   ** ===================================================================
//   */
//   state[0].Pos.z = (dT() * dT() / Mass()) * ( Mass() * G) + 2 * state[-1].Pos.z - state[-2].Pos.z;
//   state[0].Vel.z = (state[0].Pos.z - state[-1].Pos.z) / dT();
// }

void FourWheelRobot::ExtraDraw(void) {
  const IGeometryCreator *geo;
  geo = getGui()->getSceneManager()->getGeometryCreator();
  
  ITexture *texture = getGui()->getTexture("ArUco_id_3_4x4.png");// if the aruco is not working place image inside flair/flair-src/models. if you want to use another image, change the path here and in CMakeLists.txt
  IMesh* plane= geo->createPlaneMesh(irr::core::dimension2df(90,90));
  MeshSceneNode* tag= new MeshSceneNode(this, plane, vector3df(0, 0, 90),vector3df(90, 0, 0),texture);
  //creation d'un fond sinon le tag est transparent lorsqu'il est vu de l'autre coté
  // MeshSceneNode* fond= new MeshSceneNode(this, plane, vector3df(-150, 0, 0),vector3df(0, 0, 0));
  
  // texture = getGui()->getTexture("carbone.jpg");
  // IMesh *black_cyl = geo->createCylinderMesh(2.5, -150, 16, SColor(0, 128, 128, 128));
  // MeshSceneNode *tag_arm = new MeshSceneNode(this, black_cyl, vector3df(0, 0, 0),vector3df(0, 0, -90),texture);
}


} // end namespace simulator
} // end namespace flair