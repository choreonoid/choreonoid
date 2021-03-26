#ifndef CNOID_SCENE_EFFECTS_PLUGIN_SCENE_EFFECT_DEVICE_TYPE_REGISTRATION_H
#define CNOID_SCENE_EFFECTS_PLUGIN_SCENE_EFFECT_DEVICE_TYPE_REGISTRATION_H

#include <cnoid/StdBodyLoader>
#include <cnoid/StdBodyWriter>
#include <cnoid/SceneDevice>

namespace cnoid {

template <class DeviceType, class SceneNodeType>
struct SceneEffectDeviceTypeRegistration
{
    SceneEffectDeviceTypeRegistration(const char* typeName)
    {
        StdBodyLoader::registerNodeType(
            typeName,
            [](StdBodyLoader* loader, const Mapping* info)
            {
                ref_ptr<DeviceType> device = new DeviceType;
                device->particleSystem().readParameters(info);
                return loader->readDevice(device, info);
            });
        
        StdBodyWriter::registerDeviceWriter<DeviceType>(
            typeName,
            [](StdBodyWriter* /* writer */, Mapping* info, const DeviceType* device)
            {
                device->particleSystem().writeParameters(info);
                return true;
            });

        SceneDevice::registerSceneDeviceFactory<DeviceType>(
            [](Device* device) -> SceneDevice* 
            {
                auto customDevice = static_cast<DeviceType*>(device);
                auto sceneNode = new SceneNodeType;
                auto sceneDevice = new SceneDevice(device, sceneNode);

                sceneDevice->setFunctionOnStateChanged(
                    [sceneNode, customDevice](){
                        sceneNode->particleSystem() = customDevice->particleSystem();
                        sceneNode->notifyUpdate();
                    });

                sceneDevice->setFunctionOnTimeChanged(
                    [sceneNode](double time){
                        sceneNode->setTime(time);
                        sceneNode->notifyUpdate();
                    });
                
                return sceneDevice;
            });
    }
};

}

#endif
