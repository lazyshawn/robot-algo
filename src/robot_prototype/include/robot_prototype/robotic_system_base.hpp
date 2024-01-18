/**
* @file   robotic_system_base.hpp
* @brief  机器人系统模板工厂
*
* 设备模板: ArmsBase, CameraBase
*/
#include <memory>
#include <vector>
#include <string>
#include <map>

/**
* @brief  产品注册抽象类
*         提供了产品对象创建的纯虚函数 CreateProduct
* @param  AbstractProduct_t 产品抽象类
*/
template <class AbstractDevice_t>
class AbstractDeviceRegistrar {
public:
  // 获取产品对象抽象接口
  virtual std::shared_ptr<AbstractDevice_t> CreateProduct() = 0;

protected:
  // 禁止外部构造和虚构, 子类的"内部"的其他函数可以调用
  AbstractDeviceRegistrar() {}
  virtual ~AbstractDeviceRegistrar() {}

private:
  // 禁止外部拷贝和赋值操作
  AbstractDeviceRegistrar(const AbstractDeviceRegistrar &);
  const AbstractDeviceRegistrar &operator=(const AbstractDeviceRegistrar &);
};

/**
* @brief  单例模板工厂类
* @param  AbstractProduct_t 抽象产品类
*/
template <class AbstractDevice_t>
class RoboticSystem {
private:
  // 禁止外部构造和虚构
  RoboticSystem() {}
  ~RoboticSystem() {}

  // 禁止外部拷贝和赋值操作
  RoboticSystem(const RoboticSystem &);
  const RoboticSystem &operator=(const RoboticSystem &);

  // 保存注册过的产品，key:产品名字 , value:产品类型
  std::map<std::string, AbstractDeviceRegistrar<AbstractDevice_t>*> m_DeviceRegistry;

public:
  // 获取工厂单例，工厂的实例是唯一的
  static RoboticSystem<AbstractDevice_t> &Instance() {
    static RoboticSystem<AbstractDevice_t> instance;
    return instance;
  }

  // 产品注册
  void RegisterProduct(AbstractDeviceRegistrar<AbstractDevice_t>* registrar, std::string name) {
    m_DeviceRegistry[name] = registrar;
  }

  // 根据名字name，获取对应具体的产品对象
  std::shared_ptr<AbstractDevice_t> GetProduct(std::string name) {
    // 从map找到已经注册过的产品，并返回产品对象
    if (m_DeviceRegistry.find(name) != m_DeviceRegistry.end()) {
      return m_DeviceRegistry[name]->CreateProduct();
    }

    // 未注册的产品，则报错未找到
    printf("Error: #GetProduct(): No product named %s in %s.\n", name.c_str(), typeid(AbstractDevice_t).name());

    return NULL;
  }
};

/**
* @brief  产品注册模板类，用于创建具体产品和从工厂里注册产品
* @param  AbstractProduct_t 抽象产品类
* @param  ConcreteProduct_t 具体产品类
*/
template <class AbstractDevice_t, class DeviceImpl_t>
class DeviceRegistrar : public AbstractDeviceRegistrar<AbstractDevice_t> {
public:
  // 构造函数，用于注册产品到工厂，只能显示调用
  explicit DeviceRegistrar(std::string name) {
    // 通过工厂单例把产品注册到对应的工厂
    RoboticSystem<AbstractDevice_t>::Instance().RegisterProduct(this, name);
  }

  // 创建具体产品对象指针
  std::shared_ptr<AbstractDevice_t> CreateProduct() {
    return std::make_shared<DeviceImpl_t>();
  }
};

/**
* @brief  抽象产品类 - 机械臂
*/
class ArmsBase {
public:
  virtual bool read_config() = 0;
  virtual void moveJ() = 0;
  virtual void moveL() = 0;
  virtual std::vector<double> get_position() = 0;
  virtual ~ArmsBase() {}
};

/**
* @brief  抽象产品类 - 相机
*/
class CameraBase {
public:
  virtual void get_picture() = 0;
  virtual ~CameraBase() {}
};

