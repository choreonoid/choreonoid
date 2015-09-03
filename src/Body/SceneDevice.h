/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_SCENE_DEVICE_H
#define CNOID_BODY_SCENE_DEVICE_H

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
        
    // for integrating new device types
    typedef boost::function<SceneDevice*(Device* device)> SceneDeviceFactory;
    template<class DeviceType>
    static void registerSceneDeviceFactory(const SceneDeviceFactory& factory) {
        registerSceneDeviceFactory_(&typeid(DeviceType), factory);
    }

    static SceneDevice* create(Device* device);

    SceneDevice(Device* device);
    SceneDevice(Device* device, SgNode* sceneNode, boost::function<void()> sceneUpdateFunction);
    
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

protected:
    ~SceneDevice();

private:
    SceneDevice(const SceneDevice& org);
    Device* device_;
    boost::function<void()> sceneUpdateFunction;
    Connection connection;

    static void registerSceneDeviceFactory_(const std::type_info* pTypeInfo, const SceneDeviceFactory& factory);
};
    
typedef ref_ptr<SceneDevice> SceneDevicePtr;

}
    
#endif
