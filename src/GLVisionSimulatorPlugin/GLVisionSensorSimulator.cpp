#include "GLVisionSensorSimulator.h"
#include "GLVisionSimulatorItem.h"
#include <cnoid/SimulatorItem>
#include <cnoid/WorldItem>
#include <cnoid/CloneMap>
#include <cnoid/MathUtil>
#include <typeindex>

using namespace std;
using namespace cnoid;

namespace {

struct FactoryEntry {
    int id;
    std::function<GLVisionSensorSimulator*(VisionSensor*)> factory;
};

std::map<std::type_index, std::list<FactoryEntry>> factoryMap;
std::atomic<int> nextFactoryId{0};

}


int GLVisionSensorSimulator::registerSimulator_
(const std::type_info& sensorType, const std::function<GLVisionSensorSimulator*(VisionSensor* sensor)>& factory)
{
    int id = nextFactoryId++;
    factoryMap[sensorType].push_back({id, factory});
    return id;
}


void GLVisionSensorSimulator::unregisterSimulator_(const std::type_info& sensorType, int handle)
{
    if(handle < 0) return;

    auto it = factoryMap.find(sensorType);
    if(it != factoryMap.end()) {
        auto& list = it->second;
        list.remove_if([handle](const FactoryEntry& entry) {
            return entry.id == handle;
        });
        if(list.empty()) {
            factoryMap.erase(it);
        }
    }
}


GLVisionSensorSimulator* GLVisionSensorSimulator::createSimulator(VisionSensor* sensor)
{
    GLVisionSensorSimulator* simulator = nullptr;

    sensor->forEachActualType(
        [sensor, &simulator](const std::type_info& type){
            auto it = factoryMap.find(type);
            if(it != factoryMap.end()){
                auto& list = it->second;
                // Try factories from last to first (LIFO)
                for(auto rit = list.rbegin(); rit != list.rend(); ++rit) {
                    simulator = rit->factory(sensor);
                    if(simulator){
                        return true;
                    }
                }
            }
            return false;
        });

    return simulator;
}


GLVisionSensorSimulator::GLVisionSensorSimulator(VisionSensor* visionSensor)
    : visionSimulatorItem_(nullptr),
      simBody(nullptr),
      bodyIndex(0),
      visionSensor_(visionSensor),
      isOpenGLInfoOutputEnabled_(false)
{

}


GLVisionSensorSimulator::~GLVisionSensorSimulator()
{
    // Stop rendering thread before destroying screens to prevent race condition
    if(sharedScene_){
        sharedScene_->terminate();
    }
}


void GLVisionSensorSimulator::setVisionSimulator
(GLVisionSimulatorItem* visionSimulatorItem, SimulationBody* simBody, int bodyIndex)
{
    visionSimulatorItem_ = visionSimulatorItem;
    this->simBody = simBody;
    this->bodyIndex = bodyIndex;
}


bool GLVisionSensorSimulator::initialize(const vector<SimulationBody*>& simBodies)
{
    if(!doInitialize(visionSimulatorItem_)){
        return false;
    }

    if(screens_.empty()){
        addScreen();
    }
    
    if(visionSimulatorItem_->isScreenThreadEnabled()){
        for(auto& screen : screens_){
            auto scene = createSensorScene(simBodies);
            if(!screen->initialize(scene, bodyIndex)){
                return false;
            }
            scenes.push_back(scene);
        }
    } else {
        sharedScene_ = createSensorScene(simBodies);
        for(auto& screen : screens_){
            if(!screen->initialize(sharedScene_, bodyIndex)){
                return false;
            }
            isOpenGLInfoOutputEnabled_ = false;
        }
        scenes.push_back(sharedScene_);
    }
        
    elapsedTime_ = 0.0;
    latency_ = std::min(cycleTime_, visionSimulatorItem_->maxLatency());
    onsetTime_ = 0.0;
    wasDeviceOn_ = false;
    isRendering_ = false;
    needToClearVisionDataByTurningOff_ = false;

    if(visionSimulatorItem_->isSensorThreadEnabled()){
        if(sharedScene_){
            startSharedRenderingThread();
        } else {
            for(auto& screen : screens_){
                screen->startRenderingThread();
            }
        }
    }

    isBestEffortMode = visionSimulatorItem_->isBestEffortMode();

    return true;
}


GLVisionSensorRenderingScreen* GLVisionSensorSimulator::addScreen()
{
    int index = screens_.size();
    auto screen = new GLVisionSensorRenderingScreen(this, index);
    screens_.push_back(screen);
    return screen;
}


GLVisionSensorScenePtr GLVisionSensorSimulator::createSensorScene(const vector<SimulationBody*>& simBodies)
{
    GLVisionSensorScenePtr scene = new GLVisionSensorScene;
    auto& cloneMap = visionSimulatorItem_->cloneMap();
    cloneMap.clear();

    for(size_t i=0; i < simBodies.size(); ++i){
        scene->addBody(simBodies[i]->body(), cloneMap);
    }

    if(visionSimulatorItem_->isEveryRenderableItemEnabled()){
        if(auto worldItem = visionSimulatorItem_->findOwnerItem<WorldItem>()){
            for(auto& item : worldItem->descendantItems()){
                auto renderable = dynamic_cast<RenderableItem*>(item.get());
                if(renderable && !dynamic_cast<BodyItem*>(item.get())){
                    if(auto node = renderable->getScene()){
                        if(!node->hasAttribute(SgObject::MetaScene)){
                            if(auto clone = node->cloneNode(cloneMap)){
                                scene->sceneRoot()->addChild(clone);
                            }
                        }
                    }
                }
            }
        }
    }

    return scene;
}


// For SENSOR_THREAD_MODE
void GLVisionSensorSimulator::startSharedRenderingThread()
{
    // This may be unnecessary
    std::unique_lock<std::mutex> lock(sharedScene_->renderingMutex());

    bool doDoneGLContextCurrent = (screens_.size() >= 2);

    auto& renderingThread = sharedScene_->renderingThread();
    renderingThread.start(
        [this, doDoneGLContextCurrent](){
            sharedScene_->concurrentRenderingLoop(
                [this, doDoneGLContextCurrent](GLVisionSensorRenderingScreen*& currentGLContextScreen){
                    render(currentGLContextScreen, doDoneGLContextCurrent); },
                [this](){ finalizeRendering(); });
        });

    for(auto& screen : screens_){
        screen->moveRenderingBufferToThread(renderingThread);
    }
}


void GLVisionSensorSimulator::moveRenderingBufferToMainThread()
{
    for(auto& screen : screens_){
        screen->moveRenderingBufferToMainThread();
    }
}


void GLVisionSensorSimulator::updateSensorScene()
{
    for(auto& scene : scenes){
        scene->updateScene(visionSimulatorItem_->currentTime());
    }
    elapsedTime_ -= cycleTime_;
}
    

void GLVisionSensorSimulator::startConcurrentRendering()
{
    isRendering_ = true;
    updateSensorScene();

    for(auto& scene : scenes){
        scene->startConcurrentRendering();
    }
}


void GLVisionSensorSimulator::render(GLVisionSensorRenderingScreen*& currentGLContextScreen, bool doDoneGLContextCurrent)
{
    for(auto& screen : screens_){
        screen->render(currentGLContextScreen);
        if(doDoneGLContextCurrent){
            screen->doneGLContextCurrent();
            currentGLContextScreen = nullptr;
        }
    }
}


void GLVisionSensorSimulator::finalizeRendering()
{
    for(auto& screen : screens_){
        screen->finalizeRendering();
    }
}


bool GLVisionSensorSimulator::waitForRenderingToFinish()
{
    for(auto& scene : scenes){
        std::unique_lock<std::mutex> lock(scene->renderingMutex());
        if(!scene->isRenderingFinished()){
            if(isBestEffortMode){
                if(elapsedTime_ > cycleTime_){
                    elapsedTime_ = cycleTime_;
                }
                return false;
            } else {
                while(!scene->isRenderingFinished()){
                    scene->renderingCondition().wait(lock);
                }
            }
        }
    }

    for(auto& scene : scenes){
        scene->setRenderingFinished(false);
    }

    isRendering_ = false;

    return true;
}


bool GLVisionSensorSimulator::waitForRenderingToFinish(std::unique_lock<std::mutex>& lock)
{
    if(!sharedScene_->isRenderingFinished()){
        if(isBestEffortMode){
            if(elapsedTime_ > cycleTime_){
                elapsedTime_ = cycleTime_;
            }
            return false;
        } else {
            auto& queueCondition = visionSimulatorItem_->queueCondition();
            while(!sharedScene_->isRenderingFinished()){
                queueCondition.wait(lock);
            }
        }
    }
    sharedScene_->setRenderingFinished(false);
    isRendering_ = false;

    return true;
}


void GLVisionSensorSimulator::clearVisionData()
{
    doClearVisionSensorData();
    
    if(visionSimulatorItem_->isVisionDataRecordingEnabled()){
        visionSensor_->notifyStateChange();
    } else {
        simBody->notifyUnrecordedDeviceStateChange(visionSensor_);
    }

    needToClearVisionDataByTurningOff_ = false;
}


void GLVisionSensorSimulator::copyVisionData()
{
    bool hasUpdatedData = true;
    for(auto& screen : screens_){
        hasUpdatedData = hasUpdatedData && screen->hasUpdatedData();
    }

    if(hasUpdatedData){
        doUpdateVisionSensorData();
        visionSensor_->setDelay(visionSimulatorItem_->currentTime() - onsetTime_);

        if(visionSimulatorItem_->isVisionDataRecordingEnabled()){
            visionSensor_->notifyStateChange();
        } else {
            simBody->notifyUnrecordedDeviceStateChange(visionSensor_);
        }
    }
}


void GLVisionSensorSimulator::finalize()
{
    if(visionSimulatorItem_->isSensorThreadEnabled()){
        for(auto& scene : scenes){
            scene->terminate();
        }
    }
    screens_.clear();
}
    
