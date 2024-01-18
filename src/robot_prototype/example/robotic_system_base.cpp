
#include <iostream>

#include "robot_prototype/robotic_system_base.hpp"

// 定义具体的设备类
class YaskawaArm : public ArmsBase {
public:
  YaskawaArm(){}

  bool read_config() {
    return true;
  }

  void moveJ(){
    std::cout << "call moveJ()" << std::endl;
  }

  void moveL(){}

  std::vector<double> get_position() {
    return {};
  }
};

class Cam : public CameraBase {
  void get_picture() {
    std::cout << "call get_picture()" << std::endl;
  }
};

int main(int argc, char** argv) {
  std::cout << "Begin:" << std::endl;
  // 注册新设备
  DeviceRegistrar<ArmsBase, YaskawaArm> yaskawaArm("sys1_yaskawa");
  DeviceRegistrar<CameraBase, Cam> yaskawaCam("sys1_yaskawa");

  // 获取设备指针
  std::shared_ptr<ArmsBase> pYaskawa = RoboticSystem<ArmsBase>::Instance().GetProduct("sys1_yaskawa");
  std::shared_ptr<CameraBase> pYaskawaCam = RoboticSystem<CameraBase>::Instance().GetProduct("sys1_yaskawa");

  if (pYaskawa) {
    pYaskawa->moveJ();
  }
  if (pYaskawaCam != nullptr) {
    pYaskawaCam->get_picture();
  }

  return 0;
}

