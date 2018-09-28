/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneDevice.h"
#include "Link.h"
#include "ForceSensor.h"
#include "RateGyroSensor.h"
#include "AccelerationSensor.h"
#include "Camera.h"
#include "RangeCamera.h"
#include "RangeSensor.h"
#include "PointLight.h"
#include "SpotLight.h"
#include <cnoid/SceneCameras>
#include <cnoid/SceneLights>
#include <typeindex>
#include <unordered_map>

using namespace std;
using namespace cnoid;

namespace {

typedef std::unordered_map<std::type_index, SceneDevice::SceneDeviceFactory> SceneDeviceFactoryMap;
SceneDeviceFactoryMap sceneDeviceFactories;


void updatePerspectiveCamera(Camera* camera, SgPerspectiveCamera* scamera)
{
    scamera->setNearClipDistance(camera->nearClipDistance());
    scamera->setFarClipDistance(camera->farClipDistance());
    scamera->setFieldOfView(camera->fieldOfView());
    scamera->notifyUpdate();
}

void updateLight(Light* light, SgLight* slight)
{
    slight->on(light->on());
    slight->setColor(light->color());
    slight->setIntensity(light->intensity());
    slight->notifyUpdate();
}

void updatePointLight(PointLight* light, SgPointLight* slight)
{
    slight->setConstantAttenuation(light->constantAttenuation());
    slight->setLinearAttenuation(light->linearAttenuation());
    slight->setQuadraticAttenuation(light->quadraticAttenuation());
    updateLight(light, slight);
}

void updateSpotLight(SpotLight* light, SgSpotLight* slight)
{
    slight->setDirection(light->direction());
    slight->setBeamWidth(light->beamWidth());
    slight->setCutOffAngle(light->cutOffAngle());
    slight->setCutOffExponent(light->cutOffExponent());
    updatePointLight(light, slight);
}

SceneDevice* createScenePerspectiveCamera(Device* device)
{
    Camera* camera = static_cast<Camera*>(device);
    SgPerspectiveCamera* scene = new SgPerspectiveCamera();
    return new SceneDevice(camera, scene, [=](){ updatePerspectiveCamera(camera, scene); });
}
        
SceneDevice* createScenePointLight(Device* device)
{
    PointLight* pointLight = static_cast<PointLight*>(device);
    SgPointLight* scene = new SgPointLight;
    return new SceneDevice(pointLight, scene, [=](){ updatePointLight(pointLight, scene); });
}

SceneDevice* createSceneSpotLight(Device* device)
{
    SpotLight* spotLight = static_cast<SpotLight*>(device);
    SgSpotLight* scene = new SgSpotLight;
    return new SceneDevice(spotLight, scene, [=](){ updateSpotLight(spotLight, scene); });
}

SceneDevice* createNullSceneDevice(Device*)
{
    return 0;
}

}


void SceneDevice::registerSceneDeviceFactory_(const std::type_info& type, const SceneDeviceFactory& factory)
{
    sceneDeviceFactories[type] = factory;
}


static bool createSceneDevice(Device* device, const std::type_info& type, SceneDevice*& out_sceneDevice)
{
    SceneDeviceFactoryMap::iterator p = sceneDeviceFactories.find(type);
    if(p != sceneDeviceFactories.end()){
        SceneDevice::SceneDeviceFactory& factory = p->second;
        out_sceneDevice = factory(device);
        if(out_sceneDevice){
            out_sceneDevice->updateScene(0.0);
            return true;
        } else {
            return false;
        }
    }
    return false;
}


SceneDevice* SceneDevice::create(Device* device)
{
    SceneDevice* sceneDevice = nullptr;
    device->forEachActualType(
        [device, &sceneDevice](const std::type_info& type){
            return createSceneDevice(device, type, sceneDevice);
        });
    return sceneDevice;
}


SceneDevice::SceneDevice(Device* device)
    : device_(device)
{
    setTransform(device->link()->Rs().transpose() * device->T_local());
}


SceneDevice::SceneDevice
(Device* device, SgNode* sceneNode,
 std::function<void()> functionOnStateChanged,
 std::function<void(double time)> functionOnTimeChanged)
    : SceneDevice(device)
{
    sceneNode->setName(device->name());
    addChild(sceneNode);
    setFunctionOnStateChanged(functionOnStateChanged);
    setFunctionOnTimeChanged(functionOnTimeChanged);
}
    


void SceneDevice::setFunctionOnStateChanged(std::function<void()> function)
{
    functionOnStateChanged = function;
}


void SceneDevice::setFunctionOnTimeChanged(std::function<void(double time)> function)
{
    functionOnTimeChanged = function;
}


SceneDevice::SceneDevice(const SceneDevice& org)
    : SgPosTransform(org)
{
    device_ = 0;
}


SceneDevice::~SceneDevice()
{
    stateChangeConnection.disconnect();
    timeChangeConnection.disconnect();
}

void SceneDevice::setSceneUpdateConnection(bool on)
{
    stateChangeConnection.disconnect();
    timeChangeConnection.disconnect();
    if(on){
        if(functionOnStateChanged){
            stateChangeConnection = device_->sigStateChanged().connect(functionOnStateChanged);
        }
        if(functionOnTimeChanged){
            timeChangeConnection = device_->sigTimeChanged().connect(functionOnTimeChanged);
        }
    }
}

namespace {

struct SceneDeviceFactoryRegistration
{
    SceneDeviceFactoryRegistration() {
        SceneDevice::registerSceneDeviceFactory<ForceSensor>(createNullSceneDevice);
        SceneDevice::registerSceneDeviceFactory<RateGyroSensor>(createNullSceneDevice);
        SceneDevice::registerSceneDeviceFactory<AccelerationSensor>(createNullSceneDevice);
        SceneDevice::registerSceneDeviceFactory<Camera>(createScenePerspectiveCamera);
        SceneDevice::registerSceneDeviceFactory<RangeCamera>(createNullSceneDevice);
        SceneDevice::registerSceneDeviceFactory<RangeSensor>(createNullSceneDevice);
        SceneDevice::registerSceneDeviceFactory<PointLight>(createScenePointLight);
        SceneDevice::registerSceneDeviceFactory<SpotLight>(createSceneSpotLight);
    }
} registration;

}


