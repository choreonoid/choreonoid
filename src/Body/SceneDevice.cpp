/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneDevice.h"
#include "Sensor.h"
#include "Camera.h"
#include "RangeSensor.h"
#include "Light.h"
#include <cnoid/SceneCamera>
#include <cnoid/SceneLight>
#include <boost/bind.hpp>
#include <map>

using namespace std;
using namespace cnoid;

namespace {

struct compare {
    bool operator ()(const std::type_info* a, const std::type_info* b) const {
        return a->before(*b);
    }
};

typedef std::map<const std::type_info*, SceneDevice::SceneDeviceFactory, compare> SceneDeviceFactoryMap;
SceneDeviceFactoryMap sceneDeviceFactories;


void updatePerspectiveCamera(Camera* camera, SgPerspectiveCamera* scamera)
{
    scamera->setNearDistance(camera->nearDistance());
    scamera->setFarDistance(camera->farDistance());
    scamera->setFieldOfView(camera->fieldOfView());
}

void updateLight(Light* light, SgLight* slight)
{
    slight->on(light->on());
    slight->setColor(light->color());
    slight->setIntensity(light->intensity());
}

void updatePointLight(PointLight* light, SgPointLight* slight)
{
    updateLight(light, slight);
    slight->setConstantAttenuation(light->constantAttenuation());
    slight->setLinearAttenuation(light->linearAttenuation());
    slight->setQuadraticAttenuation(light->quadraticAttenuation());
}

void updateSpotLight(SpotLight* light, SgSpotLight* slight)
{
    updatePointLight(light, slight);
        
    slight->setDirection(light->direction());
    slight->setBeamWidth(light->beamWidth());
    slight->setCutOffAngle(light->cutOffAngle());
}

SceneDevice* createScenePerspectiveCamera(Device* device)
{
    Camera* camera = static_cast<Camera*>(device);
    SgPerspectiveCamera* scene = new SgPerspectiveCamera();
    return new SceneDevice(camera, scene, boost::bind(updatePerspectiveCamera, camera, scene));
}
        
SceneDevice* createScenePointLight(Device* device)
{
    PointLight* pointLight = static_cast<PointLight*>(device);
    SgPointLight* scene = new SgPointLight;
    return new SceneDevice(pointLight, scene, boost::bind(updatePointLight, pointLight, scene));
}

SceneDevice* createSceneSpotLight(Device* device)
{
    SpotLight* spotLight = static_cast<SpotLight*>(device);
    SgSpotLight* scene = new SgSpotLight;
    return new SceneDevice(spotLight, scene, boost::bind(updateSpotLight, spotLight, scene));
}

SceneDevice* createNullSceneDevice(Device* device)
{
    return 0;
}

struct SceneDeviceFactoryMapInitializer
{
    SceneDeviceFactoryMapInitializer() {
        sceneDeviceFactories[&typeid(ForceSensor)]  = createNullSceneDevice;
        sceneDeviceFactories[&typeid(RateGyroSensor)]  = createNullSceneDevice;
        sceneDeviceFactories[&typeid(AccelSensor)]  = createNullSceneDevice;
        sceneDeviceFactories[&typeid(Camera)] = createScenePerspectiveCamera;
        sceneDeviceFactories[&typeid(RangeCamera)] = createNullSceneDevice;
        sceneDeviceFactories[&typeid(RangeSensor)]  = createNullSceneDevice;
        sceneDeviceFactories[&typeid(PointLight)] = createScenePointLight;
        sceneDeviceFactories[&typeid(SpotLight)]  = createSceneSpotLight;
    }
};
SceneDeviceFactoryMapInitializer initializer;


}


void SceneDevice::registerSceneDeviceFactory_(const std::type_info* pTypeInfo, const SceneDeviceFactory& factory)
{
    sceneDeviceFactories[pTypeInfo] = factory;
}


static bool createSceneDevice(Device* device, const std::type_info& type, SceneDevice*& out_sceneDevice)
{
    SceneDeviceFactoryMap::iterator p = sceneDeviceFactories.find(&type);
    if(p != sceneDeviceFactories.end()){
        SceneDevice::SceneDeviceFactory& factory = p->second;
        out_sceneDevice = factory(device);
        if(out_sceneDevice){
            out_sceneDevice->updateScene();
            return true;
        } else {
            return false;
        }
    }
    return false;
}


SceneDevice* SceneDevice::create(Device* device)
{
    SceneDevice* sceneDevice = 0;
    device->forEachActualType(boost::bind(createSceneDevice, device,  _1, boost::ref(sceneDevice)));
    return sceneDevice;
}


SceneDevice::SceneDevice(Device* device)
    : device_(device)
{
    setTransform(device->T_local());
}


SceneDevice::SceneDevice(Device* device, SgNode* sceneNode, boost::function<void()> sceneUpdateFunction)
    : device_(device)
{
    setTransform(device->T_local());
    sceneNode->setName(device->name());
    addChild(sceneNode);
    setSceneUpdateFunction(sceneUpdateFunction);
}
    
    
void SceneDevice::setSceneUpdateFunction(boost::function<void()> function)
{
    sceneUpdateFunction = function;
}


SceneDevice::SceneDevice(const SceneDevice& org)
    : SgPosTransform(org)
{
    device_ = 0;
}


SceneDevice::~SceneDevice()
{
    connection.disconnect();
}


void SceneDevice::setSceneUpdateConnection(bool on)
{
    connection.disconnect();
    if(on && sceneUpdateFunction){
        connection = device_->sigStateChanged().connect(sceneUpdateFunction);
    }
}
