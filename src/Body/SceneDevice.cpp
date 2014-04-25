/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneDevice.h"
#include <cnoid/Camera>
#include <cnoid/Light>
#include <cnoid/SceneCamera>
#include <cnoid/SceneLight>
#include <boost/bind.hpp>

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {

struct compare {
    bool operator ()(const std::type_info* a, const std::type_info* b) const {
        return a->before(*b);
    }
};

typedef std::map<const std::type_info*, SceneDevice::DeviceNodeFactory, compare> DeviceNodeFactoryMap;
DeviceNodeFactoryMap deviceNodeFactories;

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
    
SgNode* createPerspectiveCamera(SceneDevice* sdev)
{
    Camera* camera = sdev->device<Camera>();
    SgPerspectiveCamera* scamera = new SgPerspectiveCamera();
    sdev->setSceneUpdateFunction(bind(updatePerspectiveCamera, camera, scamera));
    return scamera;
}
        
SgNode* createPointLight(SceneDevice* sdev)
{
    PointLight* light = sdev->device<PointLight>();
    SgPointLight* slight = new SgPointLight();
    sdev->setSceneUpdateFunction(bind(updatePointLight, light, slight));
    return slight;
}
        
SgNode* createSpotLight(SceneDevice* sdev)
{
    SpotLight* light = sdev->device<SpotLight>();
    SgSpotLight* slight = new SgSpotLight();
    sdev->setSceneUpdateFunction(bind(updateSpotLight, light, slight));
    return slight;
}

struct DeviceNodeFactoryMapInitializer
{
    DeviceNodeFactoryMapInitializer() {
        deviceNodeFactories[&typeid(Camera)] = createPerspectiveCamera;
        deviceNodeFactories[&typeid(PointLight)] = createPointLight;
        deviceNodeFactories[&typeid(SpotLight)]  = createSpotLight;
    }
};
DeviceNodeFactoryMapInitializer initializer;
}


void SceneDevice::registerDeviceNodeFactory_(const std::type_info* pTypeInfo, const DeviceNodeFactory& factory)
{
    deviceNodeFactories[pTypeInfo] = factory;
}


SceneDevice::SceneDevice(Device* device)
    : device_(device)
{
    device->forEachActualType(bind(&SceneDevice::setDeviceNode, this, _1));
    setTransform(device->T_local());
}


bool SceneDevice::setDeviceNode(const std::type_info& type)
{
    DeviceNodeFactoryMap::iterator p = deviceNodeFactories.find(&type);
    if(p != deviceNodeFactories.end()){
        DeviceNodeFactory& factory = p->second;
        SgNode* node = factory(this);
        node->setName(device_->name());
        updateScene();
        addChild(node);
        return true;
    }
    return false;
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
