#include "GLVisionSimulatorItem.h"
#include "GLVisionSensorSimulator.h"
#include "GLCameraSimulator.h"
#include "GLRangeCameraSimulator.h"
#include "GLRangeSensorSimulator.h"
#include "GLFisheyeCameraSimulator.h"
#include <cnoid/ItemManager>
#include <cnoid/SimulatorItem>
#include <cnoid/MessageView>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include <cnoid/ValueTreeUtil>
#include <cnoid/RenderableItem>
#include <cnoid/Body>
#include <cnoid/Camera>
#include <cnoid/RangeCamera>
#include <cnoid/RangeSensor>
#include <cnoid/SceneBody>
#include <cnoid/SceneDevice>
#include <cnoid/CloneMap>
#include <cnoid/StringUtil>
#include <cnoid/Tokenizer>
#include <cnoid/Format>
#include <cnoid/EigenArchive>
#include <QThread>
#include <mutex>
#include <queue>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

string getNameListString(const vector<string>& names)
{
    string nameList;
    if(!names.empty()){
        size_t n = names.size() - 1;
        for(size_t i=0; i < n; ++i){
            nameList += names[i];
            nameList += ", ";
        }
        nameList += names.back();
    }
    return nameList;
}

bool updateNames(const string& nameListString, string& out_newNameListString, vector<string>& out_names)
{
    out_names.clear();
    for(auto& token : Tokenizer<CharSeparator<char>>(nameListString, CharSeparator<char>(","))){
        auto name = trimmed(token);
        if(!name.empty()){
            out_names.push_back(name);
        }
    }
    out_newNameListString = nameListString;
    return true;
}

class QThreadEx : public QThread
{
    std::function<void()> function;
public:
    void start(std::function<void()> function){
        this->function = function;
        QThread::start();
    }
    virtual void run(){
        function();
    }
};

int msaaLevelToSelectionIndex(int level)
{
    switch(level){
    case -1: return 0;  // System Default
    case 0:  return 1;  // Off
    case 2:  return 2;  // 2x
    case 4:  return 3;  // 4x
    case 8:  return 4;  // 8x
    case 16: return 5;  // 16x
    default: return 1;  // Off as fallback
    }
}

int selectionIndexToMsaaLevel(int index)
{
    static const int levels[] = { -1, 0, 2, 4, 8, 16 };
    if(index >= 0 && index < 6){
        return levels[index];
    }
    return 0;  // Off as fallback
}

}

namespace cnoid {

class GLVisionSimulatorItem::Impl
{
public:
    GLVisionSimulatorItem* self;
    ostream& os;
    SimulatorItem* simulatorItem;
    double worldTimeStep;
    double currentTime;
    vector<GLVisionSensorSimulatorPtr> sensorSimulators;
    vector<GLVisionSensorSimulator*> sensorSimulatorsInRendering;
    vector<GLVisionSensorSimulator*> sensorSimulatorsToTurnOff;

    bool useQueueThreadForAllSensors;
    bool useThreadsForSensors;
    bool useThreadsForScreens;
    bool isVisionDataRecordingEnabled;
    bool isQueueRenderingTerminationRequested;

    // for the single vision simulator thread rendering
    QThreadEx queueThread;
    std::condition_variable queueCondition;
    std::mutex queueMutex;
    queue<GLVisionSensorSimulator*> sensorQueue;
    
    double rangeSensorPrecisionRatio;
    double depthError;

    vector<string> bodyNames;
    string bodyNameListString;
    vector<string> sensorNames;
    string sensorNameListString;
    Selection threadMode;
    Vector3f backgroundColor;
    bool isBestEffortMode;
    bool isEveryRenderableItemEnabled;
    bool isHeadLightEnabled;
    bool isWorldLightEnabled;
    bool isAdditionalLightSetEnabled;
    double maxFrameRate;
    double maxLatency;
    CloneMap cloneMap;
    bool isAntiAliasingEnabled;
    int msaaLevel;

    Impl(GLVisionSimulatorItem* self);
    Impl(GLVisionSimulatorItem* self, const Impl& org);
    ~Impl();
    bool initializeSimulation(SimulatorItem* simulatorItem);
    void onPreDynamics();
    void queueRenderingLoop();
    void onPostDynamics();
    void getVisionDataInThreadsForSensors();
    void getVisionDataInQueueThread();
    void finalizeSimulation();
    void doPutProperties(PutPropertyFunction& putProperty);
    bool store(Archive& archive);
    bool restore(const Archive& archive);

    template<typename Type> void setProperty(Type& variable, const Type& value){
        if(value != variable){
            variable = value;
            self->notifyUpdate();
        }
    }
};

}


void GLVisionSimulatorItem::initializeClass(ExtensionManager* ext)
{
    auto& im = ext->itemManager();
    im.registerClass<GLVisionSimulatorItem, SubSimulatorItem>(N_("GLVisionSimulatorItem"));
    im.addAlias<GLVisionSimulatorItem>("GLVisionSimulatorItem", "Body");
    im.addCreationPanel<GLVisionSimulatorItem>();
}


GLVisionSimulatorItem::GLVisionSimulatorItem()
{
    impl = new Impl(this);
    setName("GLVisionSimulator");
}


GLVisionSimulatorItem::Impl::Impl(GLVisionSimulatorItem* self)
    : self(self),
      os(MessageView::instance()->cout()),
      threadMode(GLVisionSimulatorItem::N_THREAD_MODES, CNOID_GETTEXT_DOMAIN_NAME)
{
    simulatorItem = nullptr;

    isVisionDataRecordingEnabled = false;

    rangeSensorPrecisionRatio = 2.0;
    depthError = 0.0;

    backgroundColor << 0.0f, 0.0f, 0.0f;
    isBestEffortMode = true;
    isEveryRenderableItemEnabled = true;
    isHeadLightEnabled = true;
    isWorldLightEnabled = true;
    isAdditionalLightSetEnabled = true;
    maxFrameRate = 1000.0;
    maxLatency = 1.0;

    threadMode.setSymbol(GLVisionSimulatorItem::SINGLE_THREAD_MODE, N_("Single"));
    threadMode.setSymbol(GLVisionSimulatorItem::SENSOR_THREAD_MODE, N_("Sensor"));
    threadMode.setSymbol(GLVisionSimulatorItem::SCREEN_THREAD_MODE, N_("Screen"));
    threadMode.select(GLVisionSimulatorItem::SENSOR_THREAD_MODE);

    isAntiAliasingEnabled = false;
    msaaLevel = -1;  // System Default
}


GLVisionSimulatorItem::GLVisionSimulatorItem(const GLVisionSimulatorItem& org)
    : SubSimulatorItem(org)
{
    impl = new Impl(this, *org.impl);
}


GLVisionSimulatorItem::Impl::Impl(GLVisionSimulatorItem* self, const Impl& org)
    : self(self),
      os(MessageView::instance()->cout()),
      bodyNames(org.bodyNames),
      sensorNames(org.sensorNames)
{
    simulatorItem = nullptr;

    isVisionDataRecordingEnabled = org.isVisionDataRecordingEnabled;
    
    rangeSensorPrecisionRatio = org.rangeSensorPrecisionRatio;
    depthError = org.depthError;
    
    bodyNameListString = getNameListString(bodyNames);
    sensorNameListString = getNameListString(sensorNames);
    threadMode = org.threadMode;
    backgroundColor = org.backgroundColor;
    isBestEffortMode = org.isBestEffortMode;
    isEveryRenderableItemEnabled = org.isEveryRenderableItemEnabled;
    isHeadLightEnabled = org.isHeadLightEnabled;
    isWorldLightEnabled = org.isWorldLightEnabled;
    isAdditionalLightSetEnabled = org.isAdditionalLightSetEnabled;
    maxFrameRate = org.maxFrameRate;
    maxLatency = org.maxLatency;
    isAntiAliasingEnabled = org.isAntiAliasingEnabled;
    msaaLevel = org.msaaLevel;
}


Item* GLVisionSimulatorItem::doCloneItem(CloneMap* cloneMap) const
{
    return new GLVisionSimulatorItem(*this);
}


GLVisionSimulatorItem::~GLVisionSimulatorItem()
{
    delete impl;
}


GLVisionSimulatorItem::Impl::~Impl()
{

}


void GLVisionSimulatorItem::setTargetBodies(const std::string& names)
{
    updateNames(names, impl->bodyNameListString, impl->bodyNames);
    notifyUpdate();
}


void GLVisionSimulatorItem::setTargetSensors(const std::string& names)
{
    updateNames(names, impl->sensorNameListString, impl->sensorNames);
    notifyUpdate();
}


void GLVisionSimulatorItem::setMaxFrameRate(double rate)
{
    impl->setProperty(impl->maxFrameRate, rate);
}


double GLVisionSimulatorItem::maxFrameRate() const
{
    return impl->maxFrameRate;
}


void GLVisionSimulatorItem::setMaxLatency(double latency)
{
    impl->setProperty(impl->maxLatency, latency);
}


double GLVisionSimulatorItem::maxLatency() const
{
    return impl->maxLatency;
}


void GLVisionSimulatorItem::setVisionDataRecordingEnabled(bool on)
{
    impl->setProperty(impl->isVisionDataRecordingEnabled, on);
}


bool GLVisionSimulatorItem::isVisionDataRecordingEnabled() const
{
    return impl->isVisionDataRecordingEnabled;
}


void GLVisionSimulatorItem::setThreadMode(int mode)
{
    if(mode != impl->threadMode.which()){
        impl->threadMode.select(mode);
        notifyUpdate();
    }
}


bool GLVisionSimulatorItem::isSensorThreadEnabled() const
{
    return impl->useThreadsForSensors;
}


bool GLVisionSimulatorItem::isScreenThreadEnabled() const
{
    return impl->useThreadsForScreens;
}


void GLVisionSimulatorItem::setDedicatedSensorThreadsEnabled(bool on)
{
    setThreadMode(on ? SCREEN_THREAD_MODE : SINGLE_THREAD_MODE);
}


void GLVisionSimulatorItem::setBestEffortMode(bool on)
{
    impl->setProperty(impl->isBestEffortMode, on);
}


bool GLVisionSimulatorItem::isBestEffortMode() const
{
    return impl->isBestEffortMode;
}


void GLVisionSimulatorItem::setRangeSensorPrecisionRatio(double r)
{
    impl->setProperty(impl->rangeSensorPrecisionRatio, r);
}


double GLVisionSimulatorItem::rangeSensorPrecisionRatio() const
{
    return impl->rangeSensorPrecisionRatio;
}


double GLVisionSimulatorItem::depthError() const
{
    return impl->depthError;
}


void GLVisionSimulatorItem::setBackgroundColor(const Vector3f& c)
{
    impl->backgroundColor = c;
}


const Vector3f& GLVisionSimulatorItem::backgroundColor() const
{
    return impl->backgroundColor;
}


void GLVisionSimulatorItem::setEveryRenderableItemEnabled(bool on)
{
    impl->setProperty(impl->isEveryRenderableItemEnabled, on);
}


void GLVisionSimulatorItem::setAllSceneObjectsEnabled(bool on)
{
    impl->setProperty(impl->isEveryRenderableItemEnabled, on);
}


bool GLVisionSimulatorItem::isEveryRenderableItemEnabled() const
{
    return impl->isEveryRenderableItemEnabled;
}


void GLVisionSimulatorItem::setHeadLightEnabled(bool on)
{
    impl->setProperty(impl->isHeadLightEnabled, on);
}


bool GLVisionSimulatorItem::isHeadLightEnabled() const
{
    return impl->isHeadLightEnabled;
}


void GLVisionSimulatorItem::setWorldLightEnabled(bool on)
{
    impl->setProperty(impl->isWorldLightEnabled, on);
}


bool GLVisionSimulatorItem::isWorldLightEnabled() const
{
    return impl->isWorldLightEnabled;
}


void GLVisionSimulatorItem::setAdditionalLightSetEnabled(bool on)
{
    impl->setProperty(impl->isAdditionalLightSetEnabled, on);
}


void GLVisionSimulatorItem::setAdditionalLightsEnabled(bool on)
{
    impl->setProperty(impl->isAdditionalLightSetEnabled, on);
}


bool GLVisionSimulatorItem::isAdditionalLightSetEnabled() const
{
    return impl->isAdditionalLightSetEnabled;
}


bool GLVisionSimulatorItem::isAntiAliasingEnabled() const
{
    return impl->isAntiAliasingEnabled;
}


int GLVisionSimulatorItem::msaaLevel() const
{
    return impl->msaaLevel;
}


void GLVisionSimulatorItem::setMsaaLevel(int level)
{
    impl->setProperty(impl->msaaLevel, level);
}


CloneMap& GLVisionSimulatorItem::cloneMap()
{
    return impl->cloneMap;
}


double GLVisionSimulatorItem::currentTime() const
{
    return impl->currentTime;
}


std::condition_variable& GLVisionSimulatorItem::queueCondition()
{
    return impl->queueCondition;
}


bool GLVisionSimulatorItem::initializeSimulation(SimulatorItem* simulatorItem)
{
    return impl->initializeSimulation(simulatorItem);
}


bool GLVisionSimulatorItem::Impl::initializeSimulation(SimulatorItem* simulatorItem)
{
    this->simulatorItem = simulatorItem;
    worldTimeStep = simulatorItem->worldTimeStep();
    currentTime = 0;
    sensorSimulators.clear();

    switch(threadMode.which()){
    case GLVisionSimulatorItem::SINGLE_THREAD_MODE:
        useQueueThreadForAllSensors = true;
        useThreadsForSensors = false;
        useThreadsForScreens = false;
        break;
    case GLVisionSimulatorItem::SENSOR_THREAD_MODE:
        useQueueThreadForAllSensors = false;
        useThreadsForSensors = true;
        useThreadsForScreens = false;
        break;
    case GLVisionSimulatorItem::SCREEN_THREAD_MODE:
        useQueueThreadForAllSensors = false;
        useThreadsForSensors = true;
        useThreadsForScreens = true;
        break;
    }
    
    sensorSimulatorsInRendering.clear();
    sensorSimulatorsToTurnOff.clear();

    cloneMap.clear();

    /*
      If this is set to false, rendering may crach in multi-thread rendering
      even if the non node objects in the scene are not modified because
      the signal connection / disconnection operations may collide.
    */
    SgObject::setNonNodeCloning(cloneMap, true);

    SgObject::setMetaSceneCloning(cloneMap, false);
    
    std::set<string> bodyNameSet;
    for(size_t i=0; i < bodyNames.size(); ++i){
        bodyNameSet.insert(bodyNames[i]);
    }
    std::set<string> sensorNameSet;
    for(size_t i=0; i < sensorNames.size(); ++i){
        sensorNameSet.insert(sensorNames[i]);
    }

    const vector<SimulationBody*>& simBodies = simulatorItem->simulationBodies();
    for(size_t bodyIndex = 0; bodyIndex < simBodies.size(); ++bodyIndex){
        SimulationBody* simBody = simBodies[bodyIndex];
        Body* body = simBody->body();
        if(bodyNameSet.empty() || bodyNameSet.find(body->name()) != bodyNameSet.end()){
            for(int i=0; i < body->numDevices(); ++i){
                if(auto sensor = dynamic_cast<VisionSensor*>(body->device(i))){
                    if(sensorNameSet.empty() || sensorNameSet.find(sensor->name()) != sensorNameSet.end()){
                        if(auto sensorSimulator = GLVisionSensorSimulator::createSimulator(sensor)){
                            sensorSimulator->setVisionSimulator(self, simBody, bodyIndex);
                            sensorSimulators.push_back(sensorSimulator);
                            os << formatR(_("{0} detected vision sensor \"{1}\" of {2} as a target.\n"),
                                          self->displayName(), sensor->name(), simBody->body()->name());
                        }
                    }
                }
            }
        }
    }

    if(sensorSimulators.empty()){
        os << formatR(_("{} has no target sensors"), self->displayName()) << endl;
        return false;
    }

#ifdef Q_OS_LINUX
    /**
       The following code is neccessary to avoid a crash when a view which has a widget such as
       QPlainTextEdit and has not been focused yet is first focused (clikced) during the camera
       image simulation processed by GLVisionSimulatorItem. The crash only occurs in Linux with
       the nVidia proprietary X driver. If the user clicks such a view to give the focus before
       the simulation started, the crash doesn't occur, so here the focus is forced to be given
       by the following code.

       \note The following code is disabled now because te problem does not seem to occur in
       recent Choreonoid versions and environments.
    */
    /*
    if(QWidget* textEdit = MessageView::instance()->findChild<QWidget*>("TextEdit")){
        textEdit->setFocus();
        //! todo restore the previous focus here
    }
    */
#endif
    
    bool isOpenGLInfoOutputSucceeded = false;
    auto it = sensorSimulators.begin();
    while(it != sensorSimulators.end()){
        GLVisionSensorSimulator* sensorSimulator = it->get();
        // Enable OpenGL info output for the first sensor or after previous initialization failures
        sensorSimulator->setOpenGLInfoOutputEnabled(!isOpenGLInfoOutputSucceeded);
        if(sensorSimulator->initialize(simBodies)){
            // Mark OpenGL info as successfully output if it was enabled
            if(!isOpenGLInfoOutputSucceeded){
                isOpenGLInfoOutputSucceeded = true;
            }
            ++it;
        } else {
            os << formatR(_("{0}: Target sensor \"{1}\" cannot be initialized.\n"),
                          self->displayName(), sensorSimulator->visionSensor()->name());
            it = sensorSimulators.erase(it);
        }
    }
    os.flush();

    if(!sensorSimulators.empty()){
        simulatorItem->addPreDynamicsFunction([this](){ onPreDynamics(); });
        simulatorItem->addPostDynamicsFunction([this](){ onPostDynamics(); });

        if(useQueueThreadForAllSensors){
            while(!sensorQueue.empty()){
                sensorQueue.pop();
            }
            isQueueRenderingTerminationRequested = false;
            queueThread.start([this](){ queueRenderingLoop(); });
            for(size_t i=0; i < sensorSimulators.size(); ++i){
                for(auto& screen : sensorSimulators[i]->screens()){
                    screen->moveRenderingBufferToThread(queueThread);
                }
            }
        }

        return true;
    }

    return false;
}


void GLVisionSimulatorItem::Impl::onPreDynamics()
{
    currentTime = simulatorItem->currentTime();

    std::mutex* pQueueMutex = nullptr;
    
    for(auto& sensorSimulator : sensorSimulators){
        bool isOn = sensorSimulator->visionSensor()->on();
        if(isOn){
            if(!sensorSimulator->wasDeviceOn()){
                if(sensorSimulator->needToClearVisionDataByTurningOff()){
                    sensorSimulatorsToTurnOff.erase(
                        std::find(sensorSimulatorsToTurnOff.begin(), sensorSimulatorsToTurnOff.end(), sensorSimulator));
                    sensorSimulator->needToClearVisionDataByTurningOff(false);
                }
                sensorSimulator->initializeElapsedTime();
            }
            if(sensorSimulator->elapsedTime() >= sensorSimulator->cycleTime()){
                if(!sensorSimulator->isRendering()){
                    sensorSimulator->setOnsetTime(currentTime);
                    sensorSimulator->isRendering(true);
                    if(useThreadsForSensors){
                        sensorSimulator->startConcurrentRendering();
                    } else {
                        if(!pQueueMutex){
                            pQueueMutex = &queueMutex;
                            pQueueMutex->lock();
                        }
                        sensorSimulator->updateSensorScene();
                        sensorQueue.push(sensorSimulator);
                    }
                    sensorSimulatorsInRendering.push_back(sensorSimulator);
                }
            }
        } else {
            if(sensorSimulator->wasDeviceOn()){
                sensorSimulator->needToClearVisionDataByTurningOff(true);
                sensorSimulatorsToTurnOff.push_back(sensorSimulator);
            }
        }
        sensorSimulator->addToElapsedTime(worldTimeStep);
        sensorSimulator->wasDeviceOn(isOn);
    }

    if(pQueueMutex){
        pQueueMutex->unlock();
        queueCondition.notify_all();
    }
}


void GLVisionSimulatorItem::Impl::queueRenderingLoop()
{
    GLVisionSensorSimulator* sensorSimulator = nullptr;
    GLVisionSensorRenderingScreen* currentGLContextScreen = nullptr;
    
    while(true){
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            while(true){
                if(isQueueRenderingTerminationRequested){
                    goto exitRenderingQueueLoop;
                }
                if(!sensorQueue.empty()){
                    sensorSimulator = sensorQueue.front();
                    sensorQueue.pop();
                    break;
                }
                queueCondition.wait(lock);
            }
        }
        sensorSimulator->render(currentGLContextScreen, true);
        
        {
            std::lock_guard<std::mutex> lock(queueMutex);
            sensorSimulator->sharedScene()->setRenderingFinished(true);
        }
        queueCondition.notify_all();
    }
    
exitRenderingQueueLoop:

    for(size_t i=0; i < sensorSimulators.size(); ++i){
        sensorSimulators[i]->moveRenderingBufferToMainThread();
    }
}


void GLVisionSimulatorItem::Impl::onPostDynamics()
{
    if(useThreadsForSensors){
        getVisionDataInThreadsForSensors();
    } else {
        getVisionDataInQueueThread();
    }

    if(!sensorSimulatorsToTurnOff.empty()){
        auto iter = sensorSimulatorsToTurnOff.begin();
        while(iter != sensorSimulatorsToTurnOff.end()){
            auto sensorSimulator = *iter;
            if(sensorSimulator->isRendering()){
                ++iter;
            } else {
                sensorSimulator->clearVisionData();
                iter = sensorSimulatorsToTurnOff.erase(iter);
            }
        }
    }
}


void GLVisionSimulatorItem::Impl::getVisionDataInThreadsForSensors()
{
    auto iter = sensorSimulatorsInRendering.begin();
    while(iter != sensorSimulatorsInRendering.end()){
        auto sensorSimulator = *iter;
        if(sensorSimulator->elapsedTime() >= sensorSimulator->latency()){
            if(sensorSimulator->waitForRenderingToFinish()){
                if(!sensorSimulator->needToClearVisionDataByTurningOff()){
                    sensorSimulator->copyVisionData();
                }
            }
        }
        if(sensorSimulator->isRendering()){
            ++iter;
        } else {
            iter = sensorSimulatorsInRendering.erase(iter);
        }
    }
}


void GLVisionSimulatorItem::Impl::getVisionDataInQueueThread()
{
    std::unique_lock<std::mutex> lock(queueMutex);
    
    auto it = sensorSimulatorsInRendering.begin();
    while(it != sensorSimulatorsInRendering.end()){
        GLVisionSensorSimulator* sensorSimulator = *it;
        if(sensorSimulator->elapsedTime() >= sensorSimulator->latency()){
            if(sensorSimulator->waitForRenderingToFinish(lock)){
                if(!sensorSimulator->needToClearVisionDataByTurningOff()){
                    sensorSimulator->copyVisionData();
                }
            }
        }
        if(sensorSimulator->isRendering()){
            ++it;
        } else {
            it = sensorSimulatorsInRendering.erase(it);
        }
    }
}


void GLVisionSimulatorItem::finalizeSimulation()
{
    impl->finalizeSimulation();
}


void GLVisionSimulatorItem::Impl::finalizeSimulation()
{
    if(useQueueThreadForAllSensors){
        {
            std::lock_guard<std::mutex> lock(queueMutex);
            isQueueRenderingTerminationRequested = true;
        }
        queueCondition.notify_all();
        queueThread.wait();
        while(!sensorQueue.empty()){
            sensorQueue.pop();
        }
    }

    for(auto& sensorSimulator : sensorSimulators){
        sensorSimulator->finalize();
    }
    sensorSimulators.clear();
}


void GLVisionSimulatorItem::doPutProperties(PutPropertyFunction& putProperty)
{
    SubSimulatorItem::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}


void GLVisionSimulatorItem::Impl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Target bodies"), bodyNameListString,
                [this](const string& names){ return updateNames(names, bodyNameListString, bodyNames); });
    putProperty(_("Target sensors"), sensorNameListString,
                [this](const string& names){ return updateNames(names, sensorNameListString, sensorNames); });
    putProperty(_("Max frame rate"), maxFrameRate, changeProperty(maxFrameRate));
    putProperty(_("Max latency [s]"), maxLatency, changeProperty(maxLatency));
    putProperty(_("Record vision data"), isVisionDataRecordingEnabled, changeProperty(isVisionDataRecordingEnabled));
    putProperty(_("Thread mode"), threadMode, [this](int index){ return threadMode.select(index); });
    putProperty(_("Best effort"), isBestEffortMode, changeProperty(isBestEffortMode));
    putProperty(_("All scene objects"), isEveryRenderableItemEnabled, changeProperty(isEveryRenderableItemEnabled));
    putProperty.min(1.0);
    putProperty(_("Precision ratio of range sensors"),
                rangeSensorPrecisionRatio, changeProperty(rangeSensorPrecisionRatio));
    putProperty.reset()(_("Depth error"), depthError, changeProperty(depthError));

    putProperty(_("Background color"), str(backgroundColor),
                [this](const string& value){
                    Vector3f c;
                    if(toVector3(value, c)){
                        backgroundColor = c;
                        return true;
                    }
                    return false;
                });
                        
    putProperty(_("Head light"), isHeadLightEnabled, changeProperty(isHeadLightEnabled));
    putProperty(_("World light"), isWorldLightEnabled, changeProperty(isWorldLightEnabled));
    putProperty(_("Additional lights"), isAdditionalLightSetEnabled, changeProperty(isAdditionalLightSetEnabled));
    putProperty(_("Anti-aliasing"), isAntiAliasingEnabled, changeProperty(isAntiAliasingEnabled));

    Selection msaaSelection = { _("System Default"), _("Off"), "2x", "4x", "8x", "16x" };
    msaaSelection.select(msaaLevelToSelectionIndex(msaaLevel));
    putProperty(_("MSAA level"), msaaSelection,
                [this](int index){
                    msaaLevel = selectionIndexToMsaaLevel(index);
                    self->notifyUpdate();
                    return true;
                });
}


bool GLVisionSimulatorItem::store(Archive& archive)
{
    SubSimulatorItem::store(archive);
    return impl->store(archive);
}


bool GLVisionSimulatorItem::Impl::store(Archive& archive)
{
    writeElements(archive, "target_bodies", bodyNames, true);
    writeElements(archive, "target_sensors", sensorNames, true);
    archive.write("max_frame_rate", maxFrameRate);
    archive.write("max_latency", maxLatency);
    archive.write("record_vision_data", isVisionDataRecordingEnabled);
    archive.write("thread_mode", threadMode.selectedSymbol());
    archive.write("best_effort", isBestEffortMode);
    archive.write("all_scene_objects", isEveryRenderableItemEnabled);
    archive.write("range_sensor_precision_ratio", rangeSensorPrecisionRatio);
    archive.write("depth_error", depthError);
    write(archive, "background_color", backgroundColor);
    archive.write("enable_head_light", isHeadLightEnabled);
    archive.write("enable_world_light", isWorldLightEnabled);
    archive.write("enable_additional_lights", isAdditionalLightSetEnabled);
    archive.write("antialiasing", isAntiAliasingEnabled);
    if(msaaLevel >= 0){
        archive.write("msaa_level", msaaLevel);
    }
    return true;
}


bool GLVisionSimulatorItem::restore(const Archive& archive)
{
    SubSimulatorItem::restore(archive);
    return impl->restore(archive);
}


bool GLVisionSimulatorItem::Impl::restore(const Archive& archive)
{
    readElements(archive, { "target_bodies", "targetBodies" }, bodyNames);
    bodyNameListString = getNameListString(bodyNames);
    readElements(archive, { "target_sensors", "targetSensors" }, sensorNames);
    sensorNameListString = getNameListString(sensorNames);

    archive.read({ "max_frame_rate", "maxFrameRate" }, maxFrameRate);
    archive.read({ "max_latency", "maxLatency" }, maxLatency);
    archive.read({ "record_vision_data", "recordVisionData" }, isVisionDataRecordingEnabled);
    archive.read({ "best_effort", "bestEffort" }, isBestEffortMode);
    archive.read({ "all_scene_objects", "allSceneObjects" }, isEveryRenderableItemEnabled);
    archive.read({ "range_sensor_precision_ratio", "rangeSensorPrecisionRatio" }, rangeSensorPrecisionRatio);
    archive.read({ "depth_error", "depthError" }, depthError);
    read(archive, "background_color", backgroundColor);
    archive.read({ "enable_head_light", "enableHeadLight" }, isHeadLightEnabled);
    archive.read("enable_world_light", isWorldLightEnabled);
    archive.read({ "enable_additional_lights", "enableAdditionalLights" }, isAdditionalLightSetEnabled);
    archive.read({ "antialiasing", "antiAliasing" }, isAntiAliasingEnabled);
    archive.read("msaa_level", msaaLevel);

    string symbol;
    if(archive.read({ "thread_mode", "threadMode" }, symbol)){
        threadMode.select(symbol);
    } else {
        // For the backward compatibility
        bool on;
        if(archive.read("useThreadsForSensors", on)){
            threadMode.select(on ?  GLVisionSimulatorItem::SCREEN_THREAD_MODE : GLVisionSimulatorItem::SINGLE_THREAD_MODE);
        }
    }
    
    return true;
}
