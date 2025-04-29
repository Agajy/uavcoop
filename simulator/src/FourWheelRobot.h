// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2020/11/20
//  filename:   FourWheelRobot.h
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    classe definissant un FourWheelRobot
//
/*********************************************************************/

#ifndef FOURWHEELROBOT_H
#define FOURWHEELROBOT_H

#include <Model.h>
#include <string>
#include <iostream>

namespace flair {
namespace core {
class Mutex;
}
namespace gui {
class DoubleSpinBox;
class SpinBox;
class PushButton;
}
namespace actuator {
class SimuUgvControls;
}
}

#ifdef GL
namespace irr {
namespace scene {
class IMesh;
}
}
#endif

namespace flair {
namespace simulator {
class Blade;

class FourWheelRobot : public Model {
public:
  FourWheelRobot(std::string name, uint32_t modelId);
  ~FourWheelRobot();

  void ExtraDraw(void);
#ifdef GL
  virtual void Draw(void);
  // virtual void ExtraDraw(void){}; 
#endif

private:
  void CalcModel(void);
#ifdef GL
  void AnimateModel(void);
  size_t dbtSize(void) const;
  void WritedbtBuf(char *dbtbuf);
  void ReaddbtBuf(char *dbtbuf);
  void CheckPushButton(void);
  irr::scene::IMesh *colored_body;
#endif
  gui::SpinBox *bodyColorR,*bodyColorG,*bodyColorB;
  gui::DoubleSpinBox *size,*m,*t_speed,*r_speed;
  actuator::SimuUgvControls *controls;
  flair::gui::PushButton *TwoWheelsButton, *FourWheelsButton;

  float speed;
  std::string model;
 
};
} // end namespace simulator
} // end namespace flair
#endif // TWOWHEELROBOT_H
