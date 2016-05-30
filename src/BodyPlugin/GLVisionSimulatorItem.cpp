/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "GLVisionSimulatorItem.h"
#include "SimulatorItem.h"
#include "WorldItem.h"
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <cnoid/Archive>
#include <cnoid/ValueTreeUtil>
#include <cnoid/GL1SceneRenderer>
#include <cnoid/GLSLSceneRenderer>
#include <cnoid/Body>
#include <cnoid/Camera>
#include <cnoid/RangeCamera>
#include <cnoid/RangeSensor>
#include <cnoid/SceneBody>
#include <cnoid/SceneDevice>
#include <cnoid/SceneCameras>
#include <cnoid/SceneLights>
#include <cnoid/EigenUtil>
#include <QGLPixelBuffer>
#include <boost/thread.hpp>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include <queue>
#include "gettext.h"

#include <iostream>

using namespace std;
using namespace cnoid;
using boost::format;

namespace {

inline double myNearByInt(double x)
{
#ifdef Q_OS_WIN32
    double u = ceil(x);
    double l = floor(x);
    if(fabs(u - x) < fabs(x - l)){
        return u;
    } else {
        return l;
    }
#else
    return nearbyint(x);
#endif
}

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

bool updateNames(const string& nameListString, string& newNameListString, vector<string>& names)
{
    using boost::tokenizer;
    using boost::char_separator;
    
    names.clear();
    char_separator<char> sep(",");
    tokenizer< char_separator<char> > tok(nameListString, sep);
    for(tokenizer< char_separator<char> >::iterator p = tok.begin(); p != tok.end(); ++p){
        string name = boost::trim_copy(*p);
        if(!name.empty()){
            names.push_back(name);
        }
    }
    newNameListString = nameListString;
    return true;
}


class VisionRenderer : public Referenced
{
public:
    GLVisionSimulatorItemImpl* simImpl;
    bool isRendering; // only updated and referred to in the simulation thread
    double elapsedTime;
    double cycleTime;
    double latency;
    double onsetTime;
    boost::thread renderingThread;
    boost::condition_variable renderingCondition;
    boost::mutex renderingMutex;
    bool isRenderingRequested;
    bool isRenderingFinished;
    bool isTerminationRequested;
    bool hasUpdatedData;

    DevicePtr device;
    CameraPtr camera;
    RangeCameraPtr rangeCamera;
    RangeSensorPtr rangeSensor;

    DevicePtr deviceForRendering;
    CameraPtr cameraForRendering;
    RangeCameraPtr rangeCameraForRendering;
    RangeSensorPtr rangeSensorForRendering;
    double depthError;
        
    SgGroupPtr sceneGroup;
    vector<SceneBodyPtr> sceneBodies;

    QGLPixelBuffer* pixelBuffer;
    GLSceneRenderer* renderer;
    int pixelWidth;
    int pixelHeight;
    boost::shared_ptr<Image> tmpImage;
    boost::shared_ptr<RangeCamera::PointData> tmpPoints;
    boost::shared_ptr<RangeSensor::RangeData> tmpRangeData;
    SimulationBody* simBody;
    int bodyIndex;

    VisionRenderer(GLVisionSimulatorItemImpl* simImpl, Device* sensor, SimulationBody* simBody, int bodyIndex);
    ~VisionRenderer();
    bool initialize(const vector<SimulationBody*>& simBodies);
    void initializeScene(const vector<SimulationBody*>& simBodies);
    SgCamera* initializeCamera();
    void updateScene(bool updateSensorForRenderingThread);
    void renderInCurrenThread(bool doStoreResultToTmpDataBuffer);
    void startConcurrentRendering();
    void concurrentRenderingLoop();
    void storeResultToTmpDataBuffer();
    bool waitForRenderingToFinish();
    bool waitForRenderingToFinish(boost::unique_lock<boost::mutex>& lock);
    void copyVisionData();
    void updateVisionData();
    bool getCameraImage(Image& image);
    bool getRangeCameraData(Image& image, vector<Vector3f>& points);
    bool getRangeSensorData(vector<double>& rangeData);
};
typedef ref_ptr<VisionRenderer> VisionRendererPtr;
}

namespace cnoid {

class GLVisionSimulatorItemImpl
{
public:
    GLVisionSimulatorItem* self;
    ostream& os;
    SimulatorItem* simulatorItem;
    double worldTimeStep;
    double currentTime;
    vector<VisionRendererPtr> visionRenderers;
    vector<VisionRenderer*> renderersInRendering;

    bool useGLSL;
    bool useThread;
    bool useQueueThreadForAllSensors;
    bool useThreadsForSensors;
    bool isVisionDataRecordingEnabled;
    bool isBestEffortMode;
    bool isQueueRenderingTerminationRequested;

    // for the single vision simulator thread rendering
    boost::thread queueThread;
    boost::condition_variable queueCondition;
    boost::mutex queueMutex;
    queue<VisionRenderer*> rendererQueue;
    
    double rangeSensorPrecisionRatio;
    double depthError;

    vector<string> bodyNames;
    string bodyNameListString;
    vector<string> sensorNames;
    string sensorNameListString;
    bool useThreadProperty;
    bool useThreadsForSensorsProperty;
    bool isBestEffortModeProperty;
    bool shootAllSceneObjects;
    bool isHeadLightEnabled;
    bool areAdditionalLightsEnabled;
    double maxFrameRate;
    double maxLatency;
    SgCloneMap cloneMap;
    QGLFormat glFormat;
        
    GLVisionSimulatorItemImpl(GLVisionSimulatorItem* self);
    GLVisionSimulatorItemImpl(GLVisionSimulatorItem* self, const GLVisionSimulatorItemImpl& org);
    ~GLVisionSimulatorItemImpl();
    bool initializeSimulation(SimulatorItem* simulatorItem);
    void addTargetSensor(SimulationBody* simBody, int bodyIndex, Device* sensor);
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
    ext->itemManager().registerClass<GLVisionSimulatorItem>(N_("GLVisionSimulatorItem"));
    ext->itemManager().addCreationPanel<GLVisionSimulatorItem>();
}


GLVisionSimulatorItem::GLVisionSimulatorItem()
{
    impl = new GLVisionSimulatorItemImpl(this);
    setName("GLVisionSimulator");
}


GLVisionSimulatorItemImpl::GLVisionSimulatorItemImpl(GLVisionSimulatorItem* self)
    : self(self),
      os(MessageView::instance()->cout())
{
    simulatorItem = 0;
    maxFrameRate = 1000.0;
    maxLatency = 1.0;
    rangeSensorPrecisionRatio = 2.0;
    depthError = 0.0;

    useGLSL = (getenv("CNOID_USE_GLSL") != 0);
    isVisionDataRecordingEnabled = false;
    useThreadProperty = true;
    useThreadsForSensorsProperty = true;
    isBestEffortModeProperty = false;
    isHeadLightEnabled = true;
    areAdditionalLightsEnabled = true;
    shootAllSceneObjects = false;
}


GLVisionSimulatorItem::GLVisionSimulatorItem(const GLVisionSimulatorItem& org)
    : SubSimulatorItem(org)
{
    impl = new GLVisionSimulatorItemImpl(this, *org.impl);
}


GLVisionSimulatorItemImpl::GLVisionSimulatorItemImpl(GLVisionSimulatorItem* self, const GLVisionSimulatorItemImpl& org)
    : self(self),
      os(MessageView::instance()->cout()),
      bodyNames(org.bodyNames),
      sensorNames(org.sensorNames)
{
    simulatorItem = 0;

    useGLSL = org.useGLSL;
    isVisionDataRecordingEnabled = org.isVisionDataRecordingEnabled;
    rangeSensorPrecisionRatio = org.rangeSensorPrecisionRatio;
    depthError = org.depthError;
    bodyNameListString = getNameListString(bodyNames);
    sensorNameListString = getNameListString(sensorNames);
    useThreadProperty = org.useThreadProperty;
    useThreadsForSensorsProperty = org.useThreadsForSensorsProperty;
    isBestEffortModeProperty = org.isBestEffortModeProperty;
    shootAllSceneObjects = org.shootAllSceneObjects;
    isHeadLightEnabled = org.isHeadLightEnabled;
    areAdditionalLightsEnabled = org.areAdditionalLightsEnabled;
    maxFrameRate = org.maxFrameRate;
    maxLatency = org.maxLatency;
}


Item* GLVisionSimulatorItem::doDuplicate() const
{
    return new GLVisionSimulatorItem(*this);
}


GLVisionSimulatorItem::~GLVisionSimulatorItem()
{
    delete impl;
}


GLVisionSimulatorItemImpl::~GLVisionSimulatorItemImpl()
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


void GLVisionSimulatorItem::setMaxLatency(double latency)
{
    impl->setProperty(impl->maxLatency, latency);
}


void GLVisionSimulatorItem::setVisionDataRecordingEnabled(bool on)
{
    impl->setProperty(impl->isVisionDataRecordingEnabled, on);
}


void GLVisionSimulatorItem::setThreadEnabled(bool on)
{
    impl->setProperty(impl->useThreadProperty, on);
}


void GLVisionSimulatorItem::setDedicatedSensorThreadsEnabled(bool on)
{
    impl->setProperty(impl->useThreadsForSensorsProperty, on);
}


void GLVisionSimulatorItem::setBestEffortMode(bool on)
{
    impl->setProperty(impl->isBestEffortModeProperty, on);
}


void GLVisionSimulatorItem::setRangeSensorPrecisionRatio(double r)
{
    impl->setProperty(impl->rangeSensorPrecisionRatio, r);
}


void GLVisionSimulatorItem::setAllSceneObjectsEnabled(bool on)
{
    impl->setProperty(impl->shootAllSceneObjects, on);
}


void GLVisionSimulatorItem::setHeadLightEnabled(bool on)
{
    impl->setProperty(impl->isHeadLightEnabled, on);
}


void GLVisionSimulatorItem::setAdditionalLightsEnabled(bool on)
{
    impl->setProperty(impl->areAdditionalLightsEnabled, on);
}


bool GLVisionSimulatorItem::initializeSimulation(SimulatorItem* simulatorItem)
{
    return impl->initializeSimulation(simulatorItem);
}


bool GLVisionSimulatorItemImpl::initializeSimulation(SimulatorItem* simulatorItem)
{
    if(!QGLPixelBuffer::hasOpenGLPbuffers()){
        os << (format(_("The vision sensor simulation by %1% cannot be performed because the OpenGL pbuffer is not available."))
               % self->name()) << endl;
        return false;
    }

    glFormat = QGLFormat::defaultFormat();
    glFormat.setDoubleBuffer(false);
    
    this->simulatorItem = simulatorItem;
    worldTimeStep = simulatorItem->worldTimeStep();
    currentTime = 0;
    visionRenderers.clear();

    useThread = useThreadProperty;
    if(useThread){
        if(useThreadsForSensorsProperty){
            useQueueThreadForAllSensors = false;
            useThreadsForSensors = true;
        } else {
            useQueueThreadForAllSensors = true;
            useThreadsForSensors = false;
        }
    } else {
        useQueueThreadForAllSensors = false;
        useThreadsForSensors = false;
    }
    
    isBestEffortMode = isBestEffortModeProperty;
    renderersInRendering.clear();

    cloneMap.clear();
#ifdef CNOID_REFERENCED_USE_ATOMIC_COUNTER
    cloneMap.setNonNodeCloning(false);
    cout << "cloneMap.setNonNodeCloning(false);" << endl;
#else
    cloneMap.setNonNodeCloning(true);
    cout << "cloneMap.setNonNodeCloning(true);" << endl;
#endif

    std::set<string> bodyNameSet;
    for(size_t i=0; i < bodyNames.size(); ++i){
        bodyNameSet.insert(bodyNames[i]);
    }
    std::set<string> sensorNameSet;
    for(size_t i=0; i < sensorNames.size(); ++i){
        sensorNameSet.insert(sensorNames[i]);
    }

    const vector<SimulationBody*>& simBodies = simulatorItem->simulationBodies();
    for(size_t i=0; i < simBodies.size(); ++i){
        SimulationBody* simBody = simBodies[i];
        Body* body = simBody->body();
        if(bodyNameSet.empty() || bodyNameSet.find(body->name()) != bodyNameSet.end()){
            for(size_t j=0; j < body->numDevices(); ++j){
                Device* device = body->device(j);
                if(dynamic_cast<Camera*>(device) || dynamic_cast<RangeSensor*>(device)){
                    if(sensorNameSet.empty() || sensorNameSet.find(device->name()) != sensorNameSet.end()){
                        addTargetSensor(simBody, i, device);
                    }
                }
            }
        }
    }

    if(visionRenderers.empty()){
        os << (format(_("%1% has no target sensors")) % self->name()) << endl;
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
    */
    if(QWidget* textEdit = MessageView::instance()->findChild<QWidget*>("TextEdit")){
        textEdit->setFocus();
        //! todo restore the previous focus here
    }
#endif
    
    vector<VisionRendererPtr>::iterator p = visionRenderers.begin();
    while(p != visionRenderers.end()){
        VisionRenderer* renderer = p->get();
        if(renderer->initialize(simBodies)){
            ++p;
        } else {
            os << (format(_("%1%: Target sensor \"%2%\" cannot be initialized."))
                   % self->name() % renderer->device->name()) << endl;
            p = visionRenderers.erase(p);
        }
    }

    if(!visionRenderers.empty()){
        simulatorItem->addPreDynamicsFunction(boost::bind(&GLVisionSimulatorItemImpl::onPreDynamics, this));
        simulatorItem->addPostDynamicsFunction(boost::bind(&GLVisionSimulatorItemImpl::onPostDynamics, this));

        if(useQueueThreadForAllSensors){
            while(!rendererQueue.empty()){
                rendererQueue.pop();
            }
            isQueueRenderingTerminationRequested = false;
            queueThread = boost::thread(
                boost::bind(&GLVisionSimulatorItemImpl::queueRenderingLoop, this));
        }

        return true;
    }

    return false;
}


void GLVisionSimulatorItemImpl::addTargetSensor(SimulationBody* simBody, int bodyIndex, Device* device)
{
    os << (format(_("%1% detected vision sensor \"%2%\" of %3% as a target."))
           % self->name() % device->name() % simBody->body()->name()) << endl;
    
    visionRenderers.push_back(new VisionRenderer(this, device, simBody, bodyIndex));
}


VisionRenderer::VisionRenderer(GLVisionSimulatorItemImpl* simImpl, Device* device, SimulationBody* simBody, int bodyIndex)
    : simImpl(simImpl),
      device(device),
      simBody(simBody),
      bodyIndex(bodyIndex)
{
    deviceForRendering = device->clone();
    
    camera = dynamic_cast<Camera*>(device);
    rangeCamera = dynamic_pointer_cast<RangeCamera>(camera);
    rangeSensor = dynamic_cast<RangeSensor*>(device);

    if(simImpl->useThread){
        cameraForRendering = dynamic_pointer_cast<Camera>(deviceForRendering);
        rangeCameraForRendering = dynamic_pointer_cast<RangeCamera>(deviceForRendering);
        rangeSensorForRendering = dynamic_pointer_cast<RangeSensor>(deviceForRendering);
    } else {
        cameraForRendering = camera;
        rangeCameraForRendering = rangeCamera;
        rangeSensorForRendering = rangeSensor;
    }

    pixelBuffer = 0;
    renderer = 0;
}


VisionRenderer::~VisionRenderer()
{
    if(simImpl->useThreadsForSensors){
        {
            boost::unique_lock<boost::mutex> lock(renderingMutex);
            isTerminationRequested = true;
        }
        renderingCondition.notify_all();
        renderingThread.join();
    }
    if(pixelBuffer){
        pixelBuffer->makeCurrent();
        delete pixelBuffer;
    }
    if(renderer){
        delete renderer;
    }
}


bool VisionRenderer::initialize(const vector<SimulationBody*>& simBodies)
{
    initializeScene(simBodies);

    SgCamera* sceneCamera = initializeCamera();

    if(!sceneCamera){
        return false;
    }

    if(!renderer){
        if(simImpl->useGLSL){
            renderer = new GLSLSceneRenderer;
        } else {
            renderer = new GL1SceneRenderer;
        }
    }
    
    renderer->sceneRoot()->addChild(sceneGroup);
    
    pixelBuffer = new QGLPixelBuffer(pixelWidth, pixelHeight, simImpl->glFormat);
    pixelBuffer->makeCurrent();

    renderer->initializeGL();
    renderer->setViewport(0, 0, pixelWidth, pixelHeight);
    renderer->extractPreprocessedNodes();

    if(rangeSensor){
        renderer->setDefaultLighting(false);
    } else {
        renderer->headLight()->on(simImpl->isHeadLightEnabled);
        renderer->enableAdditionalLights(simImpl->areAdditionalLightsEnabled);
    }
    
    renderer->setCurrentCamera(sceneCamera);
    pixelBuffer->doneCurrent();

    isRendering = false;
    elapsedTime = cycleTime + 1.0e-6;
    latency = std::min(cycleTime, simImpl->maxLatency);
    onsetTime = 0.0;
    hasUpdatedData = false;

    if(simImpl->useThread){
        isRenderingRequested = false;
        isRenderingFinished = false;
        isTerminationRequested = false;
        if(simImpl->useThreadsForSensors){
            renderingThread = boost::thread(
                boost::bind(&VisionRenderer::concurrentRenderingLoop, this));
        }
    }

    return true;
}


/**
   \todo use cache of the cloned scene graph nodes
*/
void VisionRenderer::initializeScene(const vector<SimulationBody*>& simBodies)
{
    sceneGroup = new SgGroup;

#ifndef CNOID_REFERENCED_USE_ATOMIC_COUNTER
    simImpl->cloneMap.clear();
#endif

    // create scene bodies
    for(size_t i=0; i < simBodies.size(); ++i){
        SceneBody* sceneBody = new SceneBody(simBodies[i]->body());

#ifndef CNOID_REFERENCED_USE_ATOMIC_COUNTER
        sceneBody->cloneShapes(simImpl->cloneMap);
#endif
        sceneBodies.push_back(sceneBody);
        sceneGroup->addChild(sceneBody);
    }

    if(simImpl->shootAllSceneObjects){
        WorldItem* worldItem = simImpl->self->findOwnerItem<WorldItem>();
        if(worldItem){
            ItemList<> items;
            items.extractChildItems(worldItem);
            for(size_t i=0; i < items.size(); ++i){
                Item* item = items.get(i);
                SceneProvider* sceneProvider = dynamic_cast<SceneProvider*>(item);
                if(sceneProvider && !dynamic_cast<BodyItem*>(item)){
                    SgNode* scene = sceneProvider->getScene(simImpl->cloneMap);
                    if(scene){
                        sceneGroup->addChild(scene);
                    }
                }
            }
        }
    }
}


SgCamera* VisionRenderer::initializeCamera()
{
    SgCamera* sceneCamera = 0;
    SceneBody* sceneBody = sceneBodies[bodyIndex];

    if(camera){
        SceneDevice* sceneDevice = sceneBody->getSceneDevice(device);
        if(sceneDevice){
            sceneCamera = sceneDevice->findNodeOfType<SgCamera>();
            pixelWidth = camera->resolutionX();
            pixelHeight = camera->resolutionY();
            double frameRate = std::max(0.1, std::min(camera->frameRate(), simImpl->maxFrameRate));
            cycleTime = 1.0 / frameRate;
            if(simImpl->isVisionDataRecordingEnabled){
                camera->setImageStateClonable(true);
            }
        }
    } else if(rangeSensor){

        const double thresh = (170.0 * PI / 180.0); // 170[deg]
        bool isWithinPossibleRanges =
            (rangeSensor->yawRange() < thresh) && (rangeSensor->pitchRange() < thresh);
        if(isWithinPossibleRanges){
            SceneLink* sceneLink = sceneBody->sceneLink(rangeSensor->link()->index());
            if(sceneLink){
                SgPerspectiveCamera* persCamera = new SgPerspectiveCamera;
                sceneCamera = persCamera;
                persCamera->setNearClipDistance(rangeSensor->minDistance());
                persCamera->setFarClipDistance(rangeSensor->maxDistance());
                SgPosTransform* cameraPos = new SgPosTransform();
                cameraPos->setTransform(rangeSensor->T_local());
                cameraPos->addChild(persCamera);
                sceneLink->addChild(cameraPos);

                if(rangeSensor->yawRange() > rangeSensor->pitchRange()){
                    pixelWidth = rangeSensor->yawResolution() * simImpl->rangeSensorPrecisionRatio;
                    if(rangeSensor->pitchRange() == 0.0){
                        pixelHeight = 1;
                        double r = tan(rangeSensor->yawRange() / 2.0) * 2.0 / pixelWidth;
                        persCamera->setFieldOfView(atan2(r / 2.0, 1.0) * 2.0);
                    } else {
                        pixelHeight = rangeSensor->pitchResolution() * simImpl->rangeSensorPrecisionRatio;
                        persCamera->setFieldOfView(rangeSensor->pitchRange());
                    }
                } else {
                    pixelHeight = rangeSensor->pitchResolution() * simImpl->rangeSensorPrecisionRatio;
                    if(rangeSensor->yawRange() == 0.0){
                        pixelWidth = 1;
                        double r = tan(rangeSensor->pitchRange() / 2.0) * 2.0 / pixelHeight;
                        persCamera->setFieldOfView(atan2(r / 2.0, 1.0) * 2.0);
                    } else {
                        pixelWidth = rangeSensor->yawResolution() * simImpl->rangeSensorPrecisionRatio;
                        persCamera->setFieldOfView(rangeSensor->yawRange());
                    }
                }
                double frameRate = std::max(0.1, std::min(rangeSensor->frameRate(), simImpl->maxFrameRate));
                cycleTime = 1.0 / frameRate;
                if(simImpl->isVisionDataRecordingEnabled){
                    rangeSensor->setRangeDataStateClonable(true);
                }

                depthError = simImpl->depthError;
            }
        }
    }

    return sceneCamera;
}


void GLVisionSimulatorItemImpl::onPreDynamics()
{
    currentTime = simulatorItem->currentTime();

    boost::mutex* pQueueMutex = 0;
    
    for(size_t i=0; i < visionRenderers.size(); ++i){
        VisionRenderer* renderer = visionRenderers[i];
        if(renderer->elapsedTime >= renderer->cycleTime){
            if(!renderer->isRendering){
                renderer->onsetTime = currentTime;
                renderer->isRendering = true;
                if(useThreadsForSensors){
                    renderer->startConcurrentRendering();
                } else if(useQueueThreadForAllSensors){
                    if(!pQueueMutex){
                        pQueueMutex = &queueMutex;
                        pQueueMutex->lock();
                    }
                    renderer->updateScene(true);
                    rendererQueue.push(renderer);
                } else {
                    renderer->updateScene(false);
                    renderer->renderInCurrenThread(false);
                }
                renderer->elapsedTime -= renderer->cycleTime;
                renderersInRendering.push_back(renderer);
            }
        }
        renderer->elapsedTime += worldTimeStep;
    }

    if(pQueueMutex){
        pQueueMutex->unlock();
        queueCondition.notify_all();
    }
}


void GLVisionSimulatorItemImpl::queueRenderingLoop()
{
    VisionRenderer* renderer = 0;
    
    while(true){
        {
            boost::unique_lock<boost::mutex> lock(queueMutex);
            while(true){
                if(isQueueRenderingTerminationRequested){
                    goto exitRenderingQueueLoop;
                }
                if(!rendererQueue.empty()){
                    renderer = rendererQueue.front();
                    rendererQueue.pop();
                    break;
                }
                queueCondition.wait(lock);
            }
        }
        renderer->renderInCurrenThread(true);
        
        {
            boost::unique_lock<boost::mutex> lock(queueMutex);
            renderer->isRenderingFinished = true;
        }
        queueCondition.notify_all();
    }
    
exitRenderingQueueLoop:
    return;
}


void VisionRenderer::updateScene(bool updateSensorForRenderingThread)
{
    for(size_t i=0; i < sceneBodies.size(); ++i){
        SceneBody* sceneBody = sceneBodies[i];
        sceneBody->updateLinkPositions();
        sceneBody->updateSceneDevices();
    }
    if(updateSensorForRenderingThread){
        deviceForRendering->copyStateFrom(*device);
    }
}


void VisionRenderer::renderInCurrenThread(bool doStoreResultToTmpDataBuffer)
{
    pixelBuffer->makeCurrent();
    renderer->render();
    renderer->flush();
    if(doStoreResultToTmpDataBuffer){
        storeResultToTmpDataBuffer();
    }
    pixelBuffer->doneCurrent();
}


void VisionRenderer::startConcurrentRendering()
{
    {
        boost::unique_lock<boost::mutex> lock(renderingMutex);
        updateScene(true);
        isRenderingRequested = true;
    }
    isRendering = true;
    renderingCondition.notify_all();
}


void VisionRenderer::concurrentRenderingLoop()
{
    pixelBuffer->makeCurrent();
    
    while(true){
        {
            boost::unique_lock<boost::mutex> lock(renderingMutex);
            while(true){
                if(isTerminationRequested){
                    goto exitConcurrentRenderingLoop;
                }
                if(isRenderingRequested){
                    isRenderingRequested = false;
                    break;
                }
                renderingCondition.wait(lock);
            }
        }
        renderer->render();
        renderer->flush();
        storeResultToTmpDataBuffer();
    
        {
            boost::unique_lock<boost::mutex> lock(renderingMutex);
            isRenderingFinished = true;
        }
        renderingCondition.notify_all();
    }
    
exitConcurrentRenderingLoop:
    pixelBuffer->doneCurrent();
    return;
}


void VisionRenderer::storeResultToTmpDataBuffer()
{
    if(cameraForRendering){
        if(!tmpImage){
            tmpImage = boost::make_shared<Image>();
        }
        if(rangeCameraForRendering){
            tmpPoints = boost::make_shared< vector<Vector3f> >();
            hasUpdatedData = getRangeCameraData(*tmpImage, *tmpPoints);
        } else {
            hasUpdatedData = getCameraImage(*tmpImage);
        }
    } else if(rangeSensorForRendering){
        tmpRangeData = boost::make_shared< vector<double> >();
        hasUpdatedData = getRangeSensorData(*tmpRangeData);
    }
}


void GLVisionSimulatorItemImpl::onPostDynamics()
{
    if(useThreadsForSensors){
        getVisionDataInThreadsForSensors();
    } else if(useQueueThreadForAllSensors){
        getVisionDataInQueueThread();
    } else {
        for(size_t i=0; i < renderersInRendering.size(); ++i){
            VisionRenderer* renderer = renderersInRendering[i];
            renderer->updateVisionData();
        }
        renderersInRendering.clear();
    }
}


void GLVisionSimulatorItemImpl::getVisionDataInThreadsForSensors()
{
    vector<VisionRenderer*>::iterator p = renderersInRendering.begin();
    while(p != renderersInRendering.end()){
        VisionRenderer* renderer = *p;
        if(renderer->elapsedTime >= renderer->latency){
            if(renderer->waitForRenderingToFinish()){
                renderer->copyVisionData();
                renderer->isRendering = false;
            }
        }
        if(renderer->isRendering){
            ++p;
        } else {
            p = renderersInRendering.erase(p);
        }
    }
}


bool VisionRenderer::waitForRenderingToFinish()
{
    boost::unique_lock<boost::mutex> lock(renderingMutex);

    if(!isRenderingFinished){
        if(simImpl->isBestEffortMode){
            if(elapsedTime > cycleTime){
                elapsedTime = cycleTime;
            }
            return false;
        } else {
            while(!isRenderingFinished){
                renderingCondition.wait(lock);
            }
        }
    }
    isRenderingFinished = false;
    return true;
}
        

void GLVisionSimulatorItemImpl::getVisionDataInQueueThread()
{
    boost::unique_lock<boost::mutex> lock(queueMutex);
    
    vector<VisionRenderer*>::iterator p = renderersInRendering.begin();
    while(p != renderersInRendering.end()){
        VisionRenderer* renderer = *p;
        if(renderer->elapsedTime >= renderer->latency){
            if(renderer->waitForRenderingToFinish(lock)){
                renderer->copyVisionData();
                renderer->isRendering = false;
            }
        }
        if(renderer->isRendering){
            ++p;
        } else {
            p = renderersInRendering.erase(p);
        }
    }
}


bool VisionRenderer::waitForRenderingToFinish(boost::unique_lock<boost::mutex>& lock)
{
    if(!isRenderingFinished){
        if(simImpl->isBestEffortMode){
            if(elapsedTime > cycleTime){
                elapsedTime = cycleTime;
            }
            return false;
        } else {
            while(!isRenderingFinished){
                simImpl->queueCondition.wait(lock);
            }
        }
    }
    isRenderingFinished = false;
    return true;
}
        

void VisionRenderer::copyVisionData()
{
    if(hasUpdatedData){
        double delay = simImpl->currentTime - onsetTime;
        if(camera){
            if(!tmpImage->empty()){
                camera->setImage(tmpImage);
            }
            if(rangeCamera){
                rangeCamera->setPoints(tmpPoints);
            }
            camera->setDelay(delay);
        } else if(rangeSensor){
            rangeSensor->setRangeData(tmpRangeData);
            rangeSensor->setDelay(delay);
        }
        if(simImpl->isVisionDataRecordingEnabled){
            device->notifyStateChange();
        } else {
            simBody->notifyUnrecordedDeviceStateChange(device);
        }
        hasUpdatedData = false;
    }
}


void VisionRenderer::updateVisionData()
{
    pixelBuffer->makeCurrent();
    bool updated = false;
    if(camera){
        if(rangeCamera){
            updated = getRangeCameraData(camera->newImage(), rangeCamera->newPoints());
        } else {
            updated = getCameraImage(camera->newImage());
        }
        if(updated){
            camera->setDelay(simImpl->currentTime - onsetTime);
        }
    } else if(rangeSensor){
        updated = getRangeSensorData(rangeSensor->newRangeData());
        if(updated){
            rangeSensor->setDelay(simImpl->currentTime - onsetTime);
        }
    }
    pixelBuffer->doneCurrent();
    
    if(updated){
        if(simImpl->isVisionDataRecordingEnabled){
            device->notifyStateChange();
        } else {
            simBody->notifyUnrecordedDeviceStateChange(device);
        }
    }
}


bool VisionRenderer::getCameraImage(Image& image)
{
    if(cameraForRendering->imageType() != Camera::COLOR_IMAGE){
        return false;
    }
    image.setSize(pixelWidth, pixelHeight, 3);
    glReadPixels(0, 0, pixelWidth, pixelHeight, GL_RGB, GL_UNSIGNED_BYTE, image.pixels());
    image.applyVerticalFlip();
    return true;
}


bool VisionRenderer::getRangeCameraData(Image& image, vector<Vector3f>& points)
{
    unsigned char* colorBuf = 0;
    unsigned char* pixels = 0;

    const bool extractColors = (cameraForRendering->imageType() == Camera::COLOR_IMAGE);
    if(extractColors){
        colorBuf = (unsigned char*)alloca(pixelWidth * pixelHeight * 3 * sizeof(unsigned char));
        glReadPixels(0, 0, pixelWidth, pixelHeight, GL_RGB, GL_UNSIGNED_BYTE, colorBuf);
        if(rangeCameraForRendering->isOrganized()){
            image.setSize(pixelWidth, pixelHeight, 3);
        } else {
            image.setSize(pixelWidth * pixelHeight, 1, 3);
        }
        pixels = image.pixels();
    }

    float* depthBuf = (float*)alloca(pixelWidth * pixelHeight * sizeof(float));
    glReadPixels(0, 0, pixelWidth, pixelHeight, GL_DEPTH_COMPONENT, GL_FLOAT, depthBuf);
    const Matrix4f Pinv = renderer->projectionMatrix().inverse().cast<float>();
    const float fw = pixelWidth;
    const float fh = pixelHeight;
    Vector4f n;
    n[3] = 1.0f;
    points.clear();
    points.reserve(pixelWidth * pixelHeight);
    unsigned char* colorSrc = 0;
    
    for(int y = pixelHeight - 1; y >= 0; --y){
        int srcpos = y * pixelWidth;
        if(extractColors){
            colorSrc = colorBuf + y * pixelWidth * 3;
        }
        for(int x=0; x < pixelWidth; ++x){
            const float z = depthBuf[srcpos + x];
            if(z > 0.0f && z < 1.0f){
                n.x() = 2.0f * x / fw - 1.0f;
                n.y() = 2.0f * y / fh - 1.0f;
                n.z() = 2.0f * z - 1.0f;
                const Vector4f o = Pinv * n;
                const float& w = o[3];
                points.push_back(Vector3f(o[0] / w, o[1] / w, o[2] / w));
                if(pixels){
                    pixels[0] = colorSrc[0];
                    pixels[1] = colorSrc[1];
                    pixels[2] = colorSrc[2];
                    pixels += 3;
                }
            } else if(rangeCameraForRendering->isOrganized()){
                if(z <= 0.0f){
                    points.push_back(Vector3f::Zero());
                } else {
                    points.push_back(Vector3f());
                    points.back() <<
                        (x - pixelWidth / 2) * numeric_limits<float>::infinity(),
                        (y - pixelWidth / 2) * numeric_limits<float>::infinity(),
                        -numeric_limits<float>::infinity();
                }
                if(pixels){
                    pixels[0] = colorSrc[0];
                    pixels[1] = colorSrc[1];
                    pixels[2] = colorSrc[2];
                    pixels += 3;
                }
            }
            colorSrc += 3;
        }
    }

    if(extractColors && !rangeCameraForRendering->isOrganized()){
        image.setSize((pixels - image.pixels()) / 3, 1, 3);
    }

    return true;
}


bool VisionRenderer::getRangeSensorData(vector<double>& rangeData)
{
    const double yawRange = rangeSensorForRendering->yawRange();
    const int yawResolution = rangeSensorForRendering->yawResolution();
    const double yawStep = rangeSensorForRendering->yawStep();
    const double maxTanYawAngle = tan(yawRange / 2.0);

    const double pitchRange = rangeSensorForRendering->pitchRange();
    const int pitchResolution = rangeSensorForRendering->pitchResolution();
    const double pitchStep = rangeSensorForRendering->pitchStep();
    const double maxTanPitchAngle = tan(pitchRange / 2.0);

    const Matrix4 Pinv = renderer->projectionMatrix().inverse();
    const double Pinv_32 = Pinv(3, 2);
    const double Pinv_33 = Pinv(3, 3);
    const double fw = pixelWidth;
    const double fh = pixelHeight;

    float* depthBuf = (float*)alloca(pixelWidth * pixelHeight * sizeof(float));
    glReadPixels(0, 0, pixelWidth, pixelHeight, GL_DEPTH_COMPONENT, GL_FLOAT, depthBuf);

    rangeData.reserve(yawResolution * pitchResolution);

    for(int pitch=0; pitch < pitchResolution; ++pitch){
        const double pitchAngle = pitch * pitchStep - pitchRange / 2.0;
        const double cosPitchAngle = cos(pitchAngle);
        int py;
        if(pitchRange == 0.0){
            py = 0;
        } else {
            const double r = (tan(pitchAngle) + maxTanPitchAngle) / (maxTanPitchAngle * 2.0);
            py = myNearByInt(r * (fh - 1.0));
        }
        const int srcpos = py * pixelWidth;

        for(int yaw=0; yaw < yawResolution; ++yaw){
            const double yawAngle = yaw * yawStep - yawRange / 2.0;
            int px;
            if(yawRange == 0.0){
                px = 0;
            } else {
                const double r = (maxTanYawAngle - tan(yawAngle)) / (maxTanYawAngle * 2.0);
                px = myNearByInt(r * (fw - 1.0));
            }
            //! \todo add the option to do the interpolation between the adjacent two pixel depths
            const float depth = depthBuf[srcpos + px];
            if(depth > 0.0f && depth < 1.0f){
                const double z0 = 2.0 * depth - 1.0;
                const double w = Pinv_32 * z0 + Pinv_33;
                const double z = -1.0 / w + depthError;
                rangeData.push_back(fabs((z / cosPitchAngle) / cos(yawAngle)));
            } else {
                rangeData.push_back(std::numeric_limits<double>::infinity());
            }
        }
    }

    return true;
}


void GLVisionSimulatorItem::finalizeSimulation()
{
    impl->finalizeSimulation();
}


void GLVisionSimulatorItemImpl::finalizeSimulation()
{
    if(useQueueThreadForAllSensors){
        {
            boost::unique_lock<boost::mutex> lock(queueMutex);
            isQueueRenderingTerminationRequested = true;
        }
        queueCondition.notify_all();
        queueThread.join();
        while(!rendererQueue.empty()){
            rendererQueue.pop();
        }
    }
        
    visionRenderers.clear();
}


void GLVisionSimulatorItem::doPutProperties(PutPropertyFunction& putProperty)
{
    SubSimulatorItem::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}


void GLVisionSimulatorItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Target bodies"), bodyNameListString, boost::bind(updateNames, _1, boost::ref(bodyNameListString), boost::ref(bodyNames)));
    putProperty(_("Target sensors"), sensorNameListString, boost::bind(updateNames, _1, boost::ref(sensorNameListString), boost::ref(sensorNames)));
    putProperty(_("Max frame rate"), maxFrameRate, changeProperty(maxFrameRate));
    putProperty(_("Max latency [s]"), maxLatency, changeProperty(maxLatency));
    putProperty(_("Record vision data"), isVisionDataRecordingEnabled, changeProperty(isVisionDataRecordingEnabled));
    putProperty(_("Use thread"), useThreadProperty, changeProperty(useThreadProperty));
    putProperty(_("Threads for sensors"), useThreadsForSensorsProperty, changeProperty(useThreadsForSensorsProperty));
    putProperty(_("Best effort"), isBestEffortModeProperty, changeProperty(isBestEffortModeProperty));
    putProperty(_("All scene objects"), shootAllSceneObjects, changeProperty(shootAllSceneObjects));
    putProperty.min(1.0)(_("Precision ratio of range sensors"),
                         rangeSensorPrecisionRatio, changeProperty(rangeSensorPrecisionRatio));
    putProperty.reset()(_("Depth error"), depthError, changeProperty(depthError));
    putProperty.reset()(_("Head light"), isHeadLightEnabled, changeProperty(isHeadLightEnabled));
    putProperty.reset()(_("Additional lights"), areAdditionalLightsEnabled, changeProperty(areAdditionalLightsEnabled));
}


bool GLVisionSimulatorItem::store(Archive& archive)
{
    SubSimulatorItem::store(archive);
    return impl->store(archive);
}


bool GLVisionSimulatorItemImpl::store(Archive& archive)
{
    writeElements(archive, "targetBodies", bodyNames, true);
    writeElements(archive, "targetSensors", sensorNames, true);
    archive.write("maxFrameRate", maxFrameRate);
    archive.write("maxLatency", maxLatency);
    archive.write("recordVisionData", isVisionDataRecordingEnabled);
    archive.write("useThread", useThreadProperty);
    archive.write("useThreadsForSensors", useThreadsForSensorsProperty);
    archive.write("bestEffort", isBestEffortModeProperty);
    archive.write("allSceneObjects", shootAllSceneObjects);
    archive.write("rangeSensorPrecisionRatio", rangeSensorPrecisionRatio);
    archive.write("depthError", depthError);
    archive.write("enableHeadLight", isHeadLightEnabled);    
    archive.write("enableAdditionalLights", areAdditionalLightsEnabled);
    return true;
}


bool GLVisionSimulatorItem::restore(const Archive& archive)
{
    SubSimulatorItem::restore(archive);
    return impl->restore(archive);
}


bool GLVisionSimulatorItemImpl::restore(const Archive& archive)
{
    readElements(archive, "targetBodies", bodyNames);
    bodyNameListString = getNameListString(bodyNames);
    readElements(archive, "targetSensors", sensorNames);
    sensorNameListString = getNameListString(sensorNames);

    archive.read("maxFrameRate", maxFrameRate);
    archive.read("maxLatency", maxLatency);
    archive.read("recordVisionData", isVisionDataRecordingEnabled);

    archive.read("useThread", useThreadProperty);
    if(!archive.read("useThreadsForSensors", useThreadsForSensorsProperty)){
        useThreadsForSensorsProperty = true; // for the compatibility. delete this later.
    }

    archive.read("bestEffort", isBestEffortModeProperty);
    archive.read("allSceneObjects", shootAllSceneObjects);
    archive.read("rangeSensorPrecisionRatio", rangeSensorPrecisionRatio);
    archive.read("depthError", depthError);
    archive.read("enableHeadLight", isHeadLightEnabled);
    archive.read("enableAdditionalLights", areAdditionalLightsEnabled);
    return true;
}
