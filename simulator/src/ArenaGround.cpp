// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2020/11/20
//  filename:   ArenaGround.cpp
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    classe definissant un ArenaGround
//
/*********************************************************************/

//  CHANGE THIS FILE TO WORK WITH 4 WHEEL ROBOT
// you need to rewrite the code for the model (CalcModel method), and the drawing code (Draw method).

#include "ArenaGround.h"
#include <TabWidget.h>
#include <Tab.h>
#include <DoubleSpinBox.h>
#include <SpinBox.h>
#include <GroupBox.h>
#include <math.h>
#include <SimuUgvControls.h>
#include "MeshSceneNode.h"
#ifdef GL
#include <ISceneManager.h>
#include <IMeshManipulator.h>
#include "MeshSceneNode.h"
#include "Gui.h"
#include <Mutex.h>
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

ArenaGround::ArenaGround(std::string name, uint32_t modelId)
    : Model(name,modelId) {

  const IGeometryCreator *geo;
  geo = getGui()->getSceneManager()->getGeometryCreator();

  colored_body = geo->createCubeMesh(vector3df(1,1,1));
  colored_body->setBoundingBox(aabbox3df(0,0,1,1,1,1));//bug with bounding box? workaround is to reduce it... we use only wheel box
  MeshSceneNode* mesh= new MeshSceneNode(this, colored_body);

  ExtraDraw();

  // geo->setFrameLoop(0, 13);
  // geo->setAnimationSpeed(0);


  Tab *setup_tab = new Tab(GetTabWidget(), "model");
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
  
  SetIsReady(true);
}

ArenaGround::~ArenaGround() {
  // les objets irrlicht seront automatiquement detruits par parenté
}

#ifdef GL

// void ArenaGround::Draw(void) {
//   // create unite (1m=100cm) robot; scale will be adapted according to settings in gcs
//   // parameter
//   // note that the frame used is irrlicht one:
//   // left handed, North East Up
//   const IGeometryCreator *geo;
//   geo = getGui()->getSceneManager()->getGeometryCreator();

//   colored_body = geo->createCubeMesh(vector3df(1,1,1));
//   colored_body->setBoundingBox(aabbox3df(0,0,0,1,1,1));//bug with bounding box? workaround is to reduce it... we use only wheel box
//   MeshSceneNode* mesh= new MeshSceneNode(this, colored_body);
  
//   // IMesh *wheel = geo->createCylinderMesh(35, 10, 64, SColor(0, 0, 0, 0));
//   // MeshSceneNode *rl_wheel = new MeshSceneNode(this, wheel, vector3df(-50, 50, -30),vector3df(0, 0, 0));
//   // MeshSceneNode *rr_wheel = new MeshSceneNode(this, wheel, vector3df(-50, -50-10, -30),vector3df(0, 0, 0));
//   // MeshSceneNode *fl_wheel = new MeshSceneNode(this, wheel, vector3df(50, 50, -30),vector3df(0, 0, 0));
//   // MeshSceneNode *fr_wheel = new MeshSceneNode(this, wheel, vector3df(50, -50-10, -30),vector3df(0, 0, 0));
  
//  ExtraDraw();
// }

void ArenaGround::AnimateModel(void) {
  if (bodyColorR->ValueChanged() == true || bodyColorG->ValueChanged() == true || bodyColorB->ValueChanged() == true) {
    getGui()->getSceneManager()->getMeshManipulator()->setVertexColors(colored_body, SColor(0,bodyColorR->Value(), bodyColorG->Value(), bodyColorB->Value()));
  }

  // adapt robot size
  if (size->ValueChanged() == true) {
    setScale(size->Value());
  }
   
}

size_t ArenaGround::dbtSize(void) const {
  return 3 * sizeof(float) + 2 * sizeof(float); // 3ddl+2motors
}

void ArenaGround::WritedbtBuf(
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

void ArenaGround::ReaddbtBuf(
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
void ArenaGround::CalcModel(void) {
  // compute quaternion from W
  // Quaternion derivative: dQ = 0.5*(Q*Qw)
  Quaternion dQ = state[-1].Quat.GetDerivative(state[0].W);

  // Quaternion integration
  state[0].Quat = state[-1].Quat + dQ * dT();
  state[0].Quat.Normalize();

  Vector3D<double> dir = state[0].Vel;
  dir.Rotate(state[0].Quat);
  state[0].Pos = state[-1].Pos + dT() * dir;
}

void ArenaGround::ExtraDraw(void) {
  const IGeometryCreator *geo;
  geo = getGui()->getSceneManager()->getGeometryCreator();
  
  ITexture *texture = getGui()->getTexture("path.jpg"); // place image inside falir/flair-src/models
  // Lors du chargement de la texture
  IMesh* plane= geo->createPlaneMesh(irr::core::dimension2df(9000,9000));
  MeshSceneNode* tag= new MeshSceneNode(this, plane, vector3df(0, 0, 1),vector3df(90, 0, 90),texture);
  // tag->setMaterialFlag(EMF_BILINEAR_FILTER, true);
  // tag->setMaterialFlag(EMF_LIGHTING, false);
  // tag->getMaterial(0).ZWriteEnable = true;
  //creation d'un fond sinon le tag est transparent lorsqu'il est vu de l'autre coté
  // MeshSceneNode* fond= new MeshSceneNode(this, plane, vector3df(-150, 0, 0),vector3df(0, 0, 0));
  
  // texture = getGui()->getTexture("carbone.jpg");
  // IMesh *black_cyl = geo->createCylinderMesh(2.5, -150, 16, SColor(0, 128, 128, 128));
  // MeshSceneNode *tag_arm = new MeshSceneNode(this, black_cyl, vector3df(0, 0, 0),vector3df(0, 0, -90),texture);
}


} // end namespace simulator
} // end namespace flair