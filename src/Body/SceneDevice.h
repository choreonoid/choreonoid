/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_SCENE_DEVICE_H_INCLUDED
#define CNOID_BODY_SCENE_DEVICE_H_INCLUDED

#include <cnoid/SceneGraph>
#include <boost/function.hpp>
#include "exportdecl.h"

namespace cnoid {

class Device;
class Camera;
class Light;
class PointLight;
class SpotLight;
    
class CNOID_EXPORT SceneDevice : public SgPosTransform
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        
    SceneDevice(Device* device);

    template <class DeviceType> DeviceType* device() {
        return static_cast<DeviceType*>(device_);
    }
    template <class DeviceType> const DeviceType* device() const {
        return static_cast<DeviceType*>(device_);
    }
    Device* device() { return device_; }
    const Device* device() const { return device_; }

    void setSceneUpdateFunction(boost::function<void()> function);
    void updateScene() { if(sceneUpdateFunction) sceneUpdateFunction(); }
    void setSceneUpdateConnection(bool on);

    // for integrating new device types
    typedef boost::function<SgNode*(SceneDevice* sdev)> DeviceNodeFactory;

    template<class DeviceType>
        static void registerDeviceNodeFactory(const DeviceNodeFactory& factory) {
        registerDeviceNodeFactory_(&typeid(factory), factory);
    }

protected:
    ~SceneDevice();

private:
    SceneDevice(const SceneDevice& org);
    Device* device_;
    boost::function<void()> sceneUpdateFunction;
    Connection connection;

    bool setDeviceNode(const std::type_info& type);

    static void registerDeviceNodeFactory_(const std::type_info* pTypeInfo, const DeviceNodeFactory& factory);
};
    
typedef ref_ptr<SceneDevice> SceneDevicePtr;
}
    
#endif
