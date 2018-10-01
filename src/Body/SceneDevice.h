/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_SCENE_DEVICE_H
#define CNOID_BODY_SCENE_DEVICE_H

#include <cnoid/SceneGraph>
#include <functional>
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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
    // for integrating new device types
    typedef std::function<SceneDevice*(Device* device)> SceneDeviceFactory;
    template<class DeviceType>
    static void registerSceneDeviceFactory(const SceneDeviceFactory& factory){
        registerSceneDeviceFactory_(typeid(DeviceType), factory);
    }

    template<class DeviceType>
    struct FactoryRegistration {
        FactoryRegistration(const SceneDeviceFactory& factory){
            registerSceneDeviceFactory_(typeid(DeviceType), factory);
        }
    };

    static SceneDevice* create(Device* device);

    SceneDevice(Device* device);
    SceneDevice(
        Device* device, SgNode* sceneNode,
        std::function<void()> functionOnStateChanged = nullptr,
        std::function<void(double time)> functionOnTimeChanged = nullptr);
    
    template <class DeviceType> DeviceType* device() {
        return static_cast<DeviceType*>(device_);
    }
    template <class DeviceType> const DeviceType* device() const {
        return static_cast<DeviceType*>(device_);
    }
    Device* device() { return device_; }
    const Device* device() const { return device_; }

    void setFunctionOnStateChanged(std::function<void()> function);
    //! @deprecated
    void setSceneUpdateFunction(std::function<void()> function){
        setFunctionOnStateChanged(function); };
    void setFunctionOnTimeChanged(std::function<void(double time)> function);
    
    void updateScene(double time) {
        if(functionOnStateChanged) functionOnStateChanged();
        if(functionOnTimeChanged) functionOnTimeChanged(time);
    }
    
    void setSceneUpdateConnection(bool on);

protected:
    ~SceneDevice();

private:
    SceneDevice(const SceneDevice& org);
    Device* device_;
    std::function<void()> functionOnStateChanged;
    std::function<void(double time)> functionOnTimeChanged;
    Connection stateChangeConnection;
    Connection timeChangeConnection;

    static void registerSceneDeviceFactory_(const std::type_info& type, const SceneDeviceFactory& factory);
};
    
typedef ref_ptr<SceneDevice> SceneDevicePtr;

}
    
#endif
