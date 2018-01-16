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
#include <QThread>
#include <QApplication>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <iostream>

static const bool DEBUG_MESSAGE = false;
static const bool DEBUG_MESSAGE2 = false;

#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#define USE_QT5_OPENGL 1
#else
#define USE_QT5_OPENGL 0
#endif

#if USE_QT5_OPENGL
#include <QOpenGLContext>
#include <QOffscreenSurface>
#include <QOpenGLFramebufferObject>
#else
#include <QGLPixelBuffer>
#endif

#include "gettext.h"

using namespace std;
using namespace cnoid;
using boost::format;

namespace {

inline int range(int i, int min, int max)
{
    return i<min ? min : i<max ? i : max-1;
}

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
    tokenizer<char_separator<char>> tok(nameListString, sep);
    for(tokenizer<char_separator<char>>::iterator p = tok.begin(); p != tok.end(); ++p){
        string name = boost::trim_copy(*p);
        if(!name.empty()){
            names.push_back(name);
        }
    }
    newNameListString = nameListString;
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

class SensorScreenRenderer;

class SensorScene : public Referenced
{
public:
    SgGroupPtr root;
    vector<SceneBodyPtr> sceneBodies;
    QThreadEx renderingThread;
    std::condition_variable renderingCondition;
    std::mutex renderingMutex;
    bool isRenderingRequested;
    bool isRenderingFinished;
    bool isTerminationRequested;

    SensorScene() {
        isRenderingRequested = false;
        isRenderingFinished = false;
        isTerminationRequested = false;
    }

    void updateScene(double currentTime);
    void startConcurrentRendering();
    void concurrentRenderingLoop(std::function<void(SensorScreenRenderer*&)> render, std::function<void()> finalizeRendering);
    void terminate();
};

typedef ref_ptr<SensorScene> SensorScenePtr;

class SensorScreenRenderer : public Referenced
{
public:
    GLVisionSimulatorItemImpl* simImpl;

    SensorScenePtr scene;

    Camera* camera;
    RangeCamera* rangeCamera;
    RangeSensor* rangeSensor;

    CameraPtr cameraForRendering;
    RangeCameraPtr rangeCameraForRendering;
    RangeSensorPtr rangeSensorForRendering;

    bool hasUpdatedData;
    double depthError;
    
#if USE_QT5_OPENGL
    QOpenGLContext* glContext;
    QOffscreenSurface* offscreenSurface;
    QOpenGLFramebufferObject* frameBuffer;
#else
    QGLPixelBuffer* renderingBuffer;
#endif

    GLSceneRenderer* renderer;
    int numYawSamples;
    int numUniqueYawSamples;
    int pixelWidth;
    int pixelHeight;
    std::shared_ptr<Image> tmpImage;
    std::shared_ptr<RangeCamera::PointData> tmpPoints;
    std::shared_ptr<RangeSensor::RangeData> tmpRangeData;
    enum ScreenId { NONE = -1, FRONT, LEFT, RIGHT, TOP, BOTTOM, BACK } screenId;

    SensorScreenRenderer(GLVisionSimulatorItemImpl* simImpl, Device* device, Device* deviceForRendering);
    ~SensorScreenRenderer();
    bool initialize(SensorScenePtr scene, int bodyIndex);
    SgCamera* initializeCamera(int bodyIndex);
    void initializeGL(SgCamera* sceneCamera);
    void startRenderingThread();
    void moveRenderingBufferToThread(QThread& thread);
    void moveRenderingBufferToMainThread();
    void makeGLContextCurrent();
    void doneGLContextCurrent();
    void updateSensorScene();
    void render(SensorScreenRenderer*& currentGLContextScreen);
    void finalizeRendering();
    void storeResultToTmpDataBuffer();
    bool getCameraImage(Image& image);
    bool getRangeCameraData(Image& image, vector<Vector3f>& points);
    bool getRangeSensorData(vector<double>& rangeData);
};
typedef ref_ptr<SensorScreenRenderer> SensorScreenRendererPtr;

class FisheyeLensConverter
{
public:
    int width;
    int height;
    double fov;
    int screenWidth;
    vector<SensorScreenRenderer*> screens;
    bool rotateOutputDirection;

    struct ScreenIndex{
        SensorScreenRenderer::ScreenId sid;
        int ix;
        int iy;
    };
    vector<vector<ScreenIndex>> fisheyeLensMap;

    // for Interpolation
    SensorScreenRenderer::ScreenId screenId[4];
    int npx[4],npy[4];
    struct ScreenIndex4 {
        ScreenIndex screenIndex[4];
        double bias[4];
    };
    vector<vector<ScreenIndex4>> fisheyeLensInterpolationMap;


    FisheyeLensConverter(){
        rotateOutputDirection = false;
    };
    void initialize(int width, int height, double fov,
            int screenWidth, vector<SensorScreenRendererPtr>& screens);
    void convertImage(Image* image);
    void convertImageInterpolation(Image* image);
    void setRotateOutputDirection(bool on){
        if(rotateOutputDirection!=on){
            fisheyeLensMap.clear();
            fisheyeLensInterpolationMap.clear();
            rotateOutputDirection = on;
        }
    };

    enum Corner { FRONT_UR, FRONT_UL, FRONT_DR, FRONT_DL, LEFT_UR, LEFT_UL, LEFT_DR, LEFT_DL,
        RIGHT_UR, RIGHT_UL, RIGHT_DR, RIGHT_DL, BOTTOM_UR, BOTTOM_UL, BOTTOM_DR, BOTTOM_DL,
        TOP_UR, TOP_UL, TOP_DR, TOP_DL, BACK_UR, BACK_UL, BACK_DR, BACK_DL};
    void setCornerPoint(int i, Corner corner){
            switch(corner){
            case FRONT_UR:
            case FRONT_UL:
            case FRONT_DR:
            case FRONT_DL:
                screenId[i] = SensorScreenRenderer::FRONT;
                break;
            case LEFT_UR:
            case LEFT_UL:
            case LEFT_DR:
            case LEFT_DL:
                screenId[i] = SensorScreenRenderer::LEFT;
                break;
            case RIGHT_UR:
            case RIGHT_UL:
            case RIGHT_DR:
            case RIGHT_DL:
                screenId[i] = SensorScreenRenderer::RIGHT;
                break;
            case BOTTOM_UR:
            case BOTTOM_UL:
            case BOTTOM_DR:
            case BOTTOM_DL:
                screenId[i] = SensorScreenRenderer::BOTTOM;
                break;
            case TOP_UR:
            case TOP_UL:
            case TOP_DR:
            case TOP_DL:
                screenId[i] = SensorScreenRenderer::TOP;
                break;
            case BACK_UR:
            case BACK_UL:
            case BACK_DR:
            case BACK_DL:
                screenId[i] = SensorScreenRenderer::BACK;
                break;
            }
            switch(corner){
            case FRONT_UL:
            case LEFT_UL:
            case RIGHT_UL:
            case BOTTOM_UL:
            case TOP_UL:
            case BACK_UL:
                npx[i] = npy[i] = 0;
                break;
            case FRONT_UR:
            case LEFT_UR:
            case RIGHT_UR:
            case BOTTOM_UR:
            case TOP_UR:
            case BACK_UR:
                npx[i] = screenWidth - 1;
                npy[i] = 0;
                break;
            case FRONT_DL:
            case LEFT_DL:
            case RIGHT_DL:
            case BOTTOM_DL:
            case TOP_DL:
            case BACK_DL:
                npx[i] = 0;
                npy[i] = screenWidth - 1;
                break;
            case FRONT_DR:
            case LEFT_DR:
            case RIGHT_DR:
            case BOTTOM_DR:
            case TOP_DR:
            case BACK_DR:
                npx[i] = screenWidth - 1;
                npy[i] = screenWidth - 1;
                break;
            }
    };
    void setCubeCorner(Corner corner0, Corner corner1, Corner corner2, Corner corner3 ){
        setCornerPoint(0, corner0);
        setCornerPoint(1, corner1);
        setCornerPoint(2, corner2);
        setCornerPoint(3, corner3);
    };
    void setCenter(SensorScreenRenderer::ScreenId id, double sx, double sy){
        screenId[0] = screenId[1] = screenId[2] = screenId[3] = id;
        npx[0] = sx;              npy[0] = sy;
        npx[1] = npx[0]+1;        npy[1] = npy[0];
        npx[2] = npx[0];          npy[2] = npy[0]+1;
        npx[3] = npx[1];          npy[3] = npy[2];
    };
    void setVerticalBorder(SensorScreenRenderer::ScreenId id0,
            SensorScreenRenderer::ScreenId id1, double sy){
        screenId[0] = screenId[2] = id0;
        screenId[1] = screenId[3] = id1;
        npx[0] = screenWidth - 1;  npy[0] = sy;
        npx[1] = 0;                npy[1] = npy[0];
        npx[2] = screenWidth - 1;  npy[2] = npy[0]+1;
        npx[3] = 0;                npy[3] = npy[2];
    };
    void setHorizontalBorder(SensorScreenRenderer::ScreenId id0,
            SensorScreenRenderer::ScreenId id1, double sx){
        screenId[0] = screenId[1] = id0;
        screenId[2] = screenId[3] = id1;
        npx[0] = sx;              npy[0] = screenWidth - 1;
        npx[1] = npx[0]+1;        npy[1] = screenWidth - 1;
        npx[2] = npx[0];          npy[2] = 0;
        npx[3] = npx[1];          npy[3] = 0;
    }
};


class SensorRenderer : public Referenced
{
public:
    GLVisionSimulatorItemImpl* simImpl;
    SimulationBody* simBody;
    int bodyIndex;
    DevicePtr device;
    CameraPtr camera;
    RangeCameraPtr rangeCamera;
    RangeSensorPtr rangeSensor;
    DevicePtr deviceForRendering;
    double elapsedTime;
    double cycleTime;
    double latency;
    double onsetTime;
    SensorScenePtr sharedScene;
    vector<SensorScenePtr> scenes;
    vector<SensorScreenRendererPtr> screens;
    bool isRendering;  // only updated and referred to in the simulation thread
    std::shared_ptr<RangeSensor::RangeData> rangeData;
    FisheyeLensConverter fisheyeLensConverter;

    SensorRenderer(GLVisionSimulatorItemImpl* simImpl, Device* sensor, SimulationBody* simBody, int bodyIndex);
    ~SensorRenderer();
    bool initialize(const vector<SimulationBody*>& simBodies);
    SensorScenePtr createSensorScene(const vector<SimulationBody*>& simBodies);
    void startSharedRenderingThread();
    void moveRenderingBufferToMainThread();
    void startConcurrentRendering();
    void updateSensorScene(bool updateSensorForRenderingThread);
    void render(SensorScreenRenderer*& currentGLContextScreen, bool doDoneGLContextCurrent);
    void finalizeRendering();
    bool waitForRenderingToFinish();
    void copyVisionData();
    bool waitForRenderingToFinish(std::unique_lock<std::mutex>& lock);
};
typedef ref_ptr<SensorRenderer> SensorRendererPtr;

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
    vector<SensorRendererPtr> sensorRenderers;
    vector<SensorRenderer*> renderersInRendering;

    bool useGLSL;
    bool useQueueThreadForAllSensors;
    bool useThreadsForSensors;
    bool useThreadsForScreens;
    bool isVisionDataRecordingEnabled;
    bool isBestEffortMode;
    bool isQueueRenderingTerminationRequested;

    // for the single vision simulator thread rendering
    QThreadEx queueThread;
    std::condition_variable queueCondition;
    std::mutex queueMutex;
    queue<SensorRenderer*> sensorQueue;
    
    double rangeSensorPrecisionRatio;
    double depthError;

    vector<string> bodyNames;
    string bodyNameListString;
    vector<string> sensorNames;
    string sensorNameListString;
    Selection threadMode;
    bool isBestEffortModeProperty;
    bool shootAllSceneObjects;
    bool isHeadLightEnabled;
    bool areAdditionalLightsEnabled;
    double maxFrameRate;
    double maxLatency;
    SgCloneMap cloneMap;
    bool rotateOutputDirection;  // for FishEyeLens
    bool enableInterpolation;
        
    GLVisionSimulatorItemImpl(GLVisionSimulatorItem* self);
    GLVisionSimulatorItemImpl(GLVisionSimulatorItem* self, const GLVisionSimulatorItemImpl& org);
    ~GLVisionSimulatorItemImpl();
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
      os(MessageView::instance()->cout()),
      threadMode(GLVisionSimulatorItem::N_THREAD_MODES, CNOID_GETTEXT_DOMAIN_NAME)
{
    simulatorItem = 0;
    maxFrameRate = 1000.0;
    maxLatency = 1.0;
    rangeSensorPrecisionRatio = 2.0;
    depthError = 0.0;

    useGLSL = (getenv("CNOID_USE_GLSL") != 0);
    isVisionDataRecordingEnabled = false;
    isBestEffortModeProperty = false;
    isHeadLightEnabled = true;
    areAdditionalLightsEnabled = true;
    shootAllSceneObjects = false;

    threadMode.setSymbol(GLVisionSimulatorItem::SINGLE_THREAD_MODE, N_("Single"));
    threadMode.setSymbol(GLVisionSimulatorItem::SENSOR_THREAD_MODE, N_("Sensor"));
    threadMode.setSymbol(GLVisionSimulatorItem::SCREEN_THREAD_MODE, N_("Screen"));
    threadMode.select(GLVisionSimulatorItem::SENSOR_THREAD_MODE);

    rotateOutputDirection = false;
    enableInterpolation = false;
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
    threadMode = org.threadMode;
    isBestEffortModeProperty = org.isBestEffortModeProperty;
    shootAllSceneObjects = org.shootAllSceneObjects;
    isHeadLightEnabled = org.isHeadLightEnabled;
    areAdditionalLightsEnabled = org.areAdditionalLightsEnabled;
    maxFrameRate = org.maxFrameRate;
    maxLatency = org.maxLatency;
    rotateOutputDirection = org.rotateOutputDirection;
    enableInterpolation = org.enableInterpolation;
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


void GLVisionSimulatorItem::setThreadMode(int mode)
{
    if(mode != impl->threadMode.which()){
        impl->threadMode.select(mode);
        notifyUpdate();
    }
}


void GLVisionSimulatorItem::setDedicatedSensorThreadsEnabled(bool on)
{
    setThreadMode(on ? SCREEN_THREAD_MODE : SINGLE_THREAD_MODE);
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
#if !USE_QT5_OPENGL
    if(!QGLPixelBuffer::hasOpenGLPbuffers()){
        os << (format(_("The vision sensor simulation by %1% cannot be performed because the OpenGL pbuffer is not available."))
               % self->name()) << endl;
        return false;
    }
#endif

    this->simulatorItem = simulatorItem;
    worldTimeStep = simulatorItem->worldTimeStep();
    currentTime = 0;
    sensorRenderers.clear();

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
    
    isBestEffortMode = isBestEffortModeProperty;
    renderersInRendering.clear();

    cloneMap.clear();

    /*
      If this is set to false, rendering may crach in multi-thread rendering
      even if the non node objects in the scene are not modified because
      the signal connection / disconnection operations may collide.
    */
    cloneMap.setNonNodeCloning(true);

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
            for(int j=0; j < body->numDevices(); ++j){
                Device* device = body->device(j);
                if(dynamic_cast<Camera*>(device) || dynamic_cast<RangeSensor*>(device)){
                    if(sensorNameSet.empty() || sensorNameSet.find(device->name()) != sensorNameSet.end()){
                        os << (format(_("%1% detected vision sensor \"%2%\" of %3% as a target."))
                               % self->name() % device->name() % simBody->body()->name()) << endl;
                        sensorRenderers.push_back(new SensorRenderer(this, device, simBody, i));
                    }
                }
            }
        }
    }

    if(sensorRenderers.empty()){
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
    
    vector<SensorRendererPtr>::iterator p = sensorRenderers.begin();
    while(p != sensorRenderers.end()){
        SensorRenderer* renderer = p->get();
        if(renderer->initialize(simBodies)){
            ++p;
        } else {
            os << (format(_("%1%: Target sensor \"%2%\" cannot be initialized."))
                   % self->name() % renderer->device->name()) << endl;
            p = sensorRenderers.erase(p);
        }
    }

    if(!sensorRenderers.empty()){
        simulatorItem->addPreDynamicsFunction([&](){ onPreDynamics(); });
        simulatorItem->addPostDynamicsFunction([&](){ onPostDynamics(); });

        if(useQueueThreadForAllSensors){
            while(!sensorQueue.empty()){
                sensorQueue.pop();
            }
            isQueueRenderingTerminationRequested = false;
            queueThread.start([&](){ queueRenderingLoop(); });
            for(size_t i=0; i < sensorRenderers.size(); ++i){
                for(auto& screen : sensorRenderers[i]->screens){
                    screen->moveRenderingBufferToThread(queueThread);
                }
            }
        }

        return true;
    }

    return false;
}


SensorRenderer::SensorRenderer(GLVisionSimulatorItemImpl* simImpl, Device* device, SimulationBody* simBody, int bodyIndex)
    : simImpl(simImpl),
      device(device),
      simBody(simBody),
      bodyIndex(bodyIndex)
{
    deviceForRendering = device->clone();
    camera = dynamic_cast<Camera*>(device);
    rangeCamera = dynamic_pointer_cast<RangeCamera>(camera);
    rangeSensor = dynamic_cast<RangeSensor*>(device);
    
    if(camera){
        if(camera->lensType()==Camera::NORMAL){
            screens.push_back(new SensorScreenRenderer(simImpl, device, deviceForRendering));
        }else{
            int numScreen = 5;
            double fov = camera->fieldOfView();
            if(fov <= radian(90)){
                numScreen = 1;
            }
            int resolution = camera->resolutionX();
            int width = resolution;
            int height = resolution;
            if(camera->lensType()==Camera::DOUBLE_FISHEYE){
                numScreen = 6;
                resolution /= 2;
                height /=2;
                fov = radian(180);
            }
            if(fov > radian(180)){
                fov = radian(180);
            }
            resolution /= 2;   //screen resolution

            Matrix3 R[6];
            R[SensorScreenRenderer::FRONT] = camera->localRotaion();
            if(numScreen > 1){
                R[SensorScreenRenderer::RIGHT] = R[SensorScreenRenderer::FRONT] * AngleAxis(radian(-90), Vector3::UnitY());
                R[SensorScreenRenderer::LEFT] = R[SensorScreenRenderer::FRONT] * AngleAxis(radian(90), Vector3::UnitY());
                R[SensorScreenRenderer::TOP] = R[SensorScreenRenderer::FRONT] * AngleAxis(radian(90), Vector3::UnitX());
                R[SensorScreenRenderer::BOTTOM] = R[SensorScreenRenderer::FRONT] * AngleAxis(radian(-90), Vector3::UnitX());
                if(numScreen==6)
                    R[SensorScreenRenderer::BACK] = R[SensorScreenRenderer::FRONT] * AngleAxis(radian(180), Vector3::UnitY());
            }
            for(int i=0; i<numScreen; i++){
                auto cameraForRendering = new Camera(*camera);
                auto screen = new SensorScreenRenderer(simImpl, device, cameraForRendering);
                screen->screenId = (SensorScreenRenderer::ScreenId)i;
                cameraForRendering->setLocalRotation(R[i]);
                cameraForRendering->setResolution(resolution,resolution);
                screens.push_back(screen);
            }

            fisheyeLensConverter.initialize(width, height, fov, resolution, screens);
            fisheyeLensConverter.setRotateOutputDirection(simImpl->rotateOutputDirection);
        }
    } else if(rangeSensor){

        int numScreens = 1;
        double yawRangePerScreen = rangeSensor->yawRange();
        if(yawRangePerScreen > radian(120.0)){
            numScreens = static_cast<int>(yawRangePerScreen / radian(91.0)) + 1;
            yawRangePerScreen = yawRangePerScreen / numScreens;
        }
        double yawRangeOffset = 0;
        double yawOffset = -rangeSensor->yawRange() / 2.0;

        const double pitchRange = std::min(rangeSensor->pitchRange(), radian(170.0));
        
        for(int i=0; i < numScreens; ++i){

            auto rangeSensorForRendering = new RangeSensor(*rangeSensor);
            auto screen = new SensorScreenRenderer(simImpl, device, rangeSensorForRendering);

            rangeSensorForRendering->setPitchRange(pitchRange);

            // Adjust to be a multiple of yawStep
            int n = 1;
            const double yawStep = rangeSensor->yawStep();
            if(yawStep > 0.0){
                n = static_cast<int>(round((yawRangePerScreen + yawRangeOffset) / yawStep)) + 1;
            }
            double adjustedYawRange = (n - 1) * yawStep;
            rangeSensorForRendering->setYawRange(adjustedYawRange);
            yawRangeOffset = yawRangePerScreen - adjustedYawRange;
            screen->numYawSamples = n;
            if(i < numScreens - 1){
                screen->numUniqueYawSamples = n - 1;
            } else {
                screen->numUniqueYawSamples = n;
            }

            double centerAngle = yawOffset + adjustedYawRange / 2.0;
            Matrix3 R = rangeSensor->localRotaion() * AngleAxis(centerAngle, Vector3::UnitY());
            rangeSensorForRendering->setLocalRotation(R);
            yawOffset += adjustedYawRange;

            screens.push_back(screen);
        }
        if(DEBUG_MESSAGE){
            cout << "Number of screens = " << numScreens << endl;
        }
    }
}


bool SensorRenderer::initialize(const vector<SimulationBody*>& simBodies)
{
    if(simImpl->useThreadsForScreens){
        for(auto& screen : screens){
            auto scene = createSensorScene(simBodies);
            if(!screen->initialize(scene, bodyIndex)){
                return false;
            }
            scenes.push_back(scene);
        }
    } else {
        sharedScene = createSensorScene(simBodies);
        for(auto& screen : screens){
            if(!screen->initialize(sharedScene, bodyIndex)){
                return false;
            }
        }
        scenes.push_back(sharedScene);
    }
        
    if(camera){
        double frameRate = std::max(0.1, std::min(camera->frameRate(), simImpl->maxFrameRate));
        cycleTime = 1.0 / frameRate;
        if(simImpl->isVisionDataRecordingEnabled){
            camera->setImageStateClonable(true);
        }
    } else if(rangeSensor){
        double frameRate = std::max(0.1, std::min(rangeSensor->scanRate(), simImpl->maxFrameRate));
        cycleTime = 1.0 / frameRate;
        if(simImpl->isVisionDataRecordingEnabled){
            rangeSensor->setRangeDataStateClonable(true);
        }
    }

    elapsedTime = cycleTime + 1.0e-6;
    latency = std::min(cycleTime, simImpl->maxLatency);
    onsetTime = 0.0;
    
    isRendering = false;

    if(simImpl->useThreadsForSensors){
        if(sharedScene){
            startSharedRenderingThread();
        } else {
            for(auto& screen : screens){
                screen->startRenderingThread();
            }
        }
    }

    return true;
}


SensorScenePtr SensorRenderer::createSensorScene(const vector<SimulationBody*>& simBodies)
{
    SensorScenePtr scene = new SensorScene;
    scene->root = new SgGroup;
    simImpl->cloneMap.clear();

    for(size_t i=0; i < simBodies.size(); ++i){
        auto sceneBody = new SceneBody(simBodies[i]->body());
        sceneBody->cloneShapes(simImpl->cloneMap);
        scene->sceneBodies.push_back(sceneBody);
        scene->root->addChild(sceneBody);
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
                    auto node = sceneProvider->cloneScene(simImpl->cloneMap);
                    if(node){
                        scene->root->addChild(node);
                    }
                }
            }
        }
    }

    return scene;
}


SensorScreenRenderer::SensorScreenRenderer(GLVisionSimulatorItemImpl* simImpl, Device* device, Device* screenDevice)
    : simImpl(simImpl)
{
    camera = dynamic_cast<Camera*>(device);
    rangeCamera = dynamic_cast<RangeCamera*>(camera);
    rangeSensor = dynamic_cast<RangeSensor*>(device);

    cameraForRendering = dynamic_cast<Camera*>(screenDevice);
    rangeCameraForRendering = dynamic_cast<RangeCamera*>(screenDevice);
    rangeSensorForRendering = dynamic_cast<RangeSensor*>(screenDevice);

#if USE_QT5_OPENGL
    glContext = 0;
    offscreenSurface = 0;
    frameBuffer = 0;
#else
    renderingBuffer = 0;
#endif

    renderer = 0;
    screenId = SensorScreenRenderer::FRONT;
}


bool SensorScreenRenderer::initialize(SensorScenePtr scene, int bodyIndex)
{
    this->scene = scene;

    auto sceneCamera = initializeCamera(bodyIndex);
    if(!sceneCamera){
        return false;
    }

    initializeGL(sceneCamera);

    hasUpdatedData = false;

    return true;
}


SgCamera* SensorScreenRenderer::initializeCamera(int bodyIndex)
{
    auto& sceneBody = scene->sceneBodies[bodyIndex];
    SgCamera* sceneCamera = nullptr;

    if(camera){
        if(camera->lensType() == Camera::NORMAL){
            auto sceneDevice = sceneBody->getSceneDevice(camera);
            if(sceneDevice){
                sceneCamera = sceneDevice->findNodeOfType<SgCamera>();
                pixelWidth = camera->resolutionX();
                pixelHeight = camera->resolutionY();
            }
        }else{
            auto sceneLink = sceneBody->sceneLink(camera->link()->index());
            if(sceneLink){
                auto persCamera = new SgPerspectiveCamera;
                sceneCamera = persCamera;
                persCamera->setNearClipDistance(cameraForRendering->nearClipDistance());
                persCamera->setFarClipDistance(cameraForRendering->farClipDistance());
                persCamera->setFieldOfView(radian(90.0));
                auto cameraPos = new SgPosTransform();
                cameraPos->setTransform(camera->link()->Rs().transpose() * cameraForRendering->T_local());
                cameraPos->addChild(persCamera);
                sceneLink->addChild(cameraPos);
                pixelWidth = cameraForRendering->resolutionX();
                pixelHeight = cameraForRendering->resolutionY();
            }
        }
    } else if(rangeSensor){
        auto sceneLink = sceneBody->sceneLink(rangeSensor->link()->index());
        if(sceneLink){
            auto persCamera = new SgPerspectiveCamera;
            sceneCamera = persCamera;
            persCamera->setNearClipDistance(rangeSensorForRendering->minDistance());
            persCamera->setFarClipDistance(rangeSensorForRendering->maxDistance());
            auto cameraPos = new SgPosTransform();
            cameraPos->setTransform(rangeSensor->link()->Rs().transpose() * rangeSensorForRendering->T_local());
            cameraPos->addChild(persCamera);
            sceneLink->addChild(cameraPos);

            const double halfYawRange = rangeSensorForRendering->yawRange() / 2.0;
            const double halfPitchRange = rangeSensorForRendering->pitchRange() / 2.0;
            const double maxTanPitch = tan(halfPitchRange) / cos(halfYawRange);
            const double maxTanYaw = tan(halfYawRange);
            const double maxPitchRange2 = atan(maxTanPitch);

            if(numYawSamples >= rangeSensorForRendering->numPitchSamples()){
                pixelWidth = numYawSamples * simImpl->rangeSensorPrecisionRatio;
                pixelHeight = pixelWidth * maxTanPitch / maxTanYaw;
            } else {
                pixelHeight = rangeSensorForRendering->numPitchSamples() * simImpl->rangeSensorPrecisionRatio;
                pixelWidth = pixelHeight * maxTanYaw / maxTanPitch;
                double r = numYawSamples * simImpl->rangeSensorPrecisionRatio;
                if(halfYawRange != 0.0 && pixelWidth < r){
                    pixelWidth = r;
                    pixelHeight = pixelWidth * maxTanPitch / maxTanYaw;
                }
            }

            if(maxTanYaw > maxTanPitch){
                if(halfPitchRange == 0.0){
                    pixelHeight = 1;
                    double r = tan(halfYawRange) * 2.0 / pixelWidth;
                    persCamera->setFieldOfView(atan2(r / 2.0, 1.0) * 2.0);
                } else {
                    persCamera->setFieldOfView(maxPitchRange2 * 2.0);
                }
            } else {
                if(halfYawRange == 0.0){
                    pixelWidth = 1;
                    double r = tan(halfPitchRange) * 2.0 / pixelHeight;
                    persCamera->setFieldOfView(atan2(r / 2.0, 1.0) * 2.0);
                } else {
                    persCamera->setFieldOfView(halfYawRange * 2.0);
                }
            }

            depthError = simImpl->depthError;

            if(DEBUG_MESSAGE){
                cout << "FieldOfView= " << degree(persCamera->fieldOfView()) << endl;
                cout << "Height= "  << pixelHeight << "  Width= "  << pixelWidth << endl;
            }
        }
    }

    return sceneCamera;
}


void SensorScreenRenderer::initializeGL(SgCamera* sceneCamera)
{
#if USE_QT5_OPENGL
    glContext = new QOpenGLContext;
    QSurfaceFormat format;
    format.setSwapBehavior(QSurfaceFormat::SingleBuffer);
    if(simImpl->useGLSL){
        format.setProfile(QSurfaceFormat::CoreProfile);
        format.setVersion(3, 3);
    }
    glContext->setFormat(format);
    glContext->create();
    offscreenSurface = new QOffscreenSurface;
    offscreenSurface->setFormat(format);
    offscreenSurface->create();
    glContext->makeCurrent(offscreenSurface);
    frameBuffer = new QOpenGLFramebufferObject(pixelWidth, pixelHeight, QOpenGLFramebufferObject::CombinedDepthStencil);
    frameBuffer->bind();
#else
    QGLFormat format;
    format.setDoubleBuffer(false);
    if(simImpl->useGLSL){
        format.setProfile(QGLFormat::CoreProfile);
        format.setVersion(3, 3);
    }
    renderingBuffer = new QGLPixelBuffer(pixelWidth, pixelHeight, format);
    renderingBuffer->makeCurrent();
#endif

    if(!renderer){
        if(simImpl->useGLSL){
            renderer = new GLSLSceneRenderer;
        } else {
            renderer = new GL1SceneRenderer;
        }
    }

    renderer->initializeGL();
    renderer->setViewport(0, 0, pixelWidth, pixelHeight);
    renderer->sceneRoot()->addChild(scene->root);
    renderer->extractPreprocessedNodes();
    renderer->setCurrentCamera(sceneCamera);

    if(rangeSensorForRendering){
        renderer->setDefaultLighting(false);
    } else {
        if(screenId != FRONT){
            SgDirectionalLight* headLight = dynamic_cast<SgDirectionalLight*>(renderer->headLight());
            if(headLight){
                switch(screenId){
                case LEFT:
                    headLight->setDirection(Vector3( 1, 0, 0));
                    break;
                case RIGHT:
                    headLight->setDirection(Vector3( -1, 0 ,0));
                    break;
                case TOP:
                    headLight->setDirection(Vector3( 0, -1 ,0));
                    break;
                case BOTTOM:
                    headLight->setDirection(Vector3( 0, 1 ,0));
                    break;
                case BACK:
                    headLight->setDirection(Vector3( 0, 0 ,1));
                    break;
                }
            }
        }
        renderer->headLight()->on(simImpl->isHeadLightEnabled);
        renderer->enableAdditionalLights(simImpl->areAdditionalLightsEnabled);
    }

    doneGLContextCurrent();
}


// For SENSOR_THREAD_MODE
void SensorRenderer::startSharedRenderingThread()
{
    // This may be unnecessary
    std::unique_lock<std::mutex> lock(sharedScene->renderingMutex);

    bool doDoneGLContextCurrent = (screens.size() >= 2);
    
    sharedScene->renderingThread.start([=](){
            sharedScene->concurrentRenderingLoop(
                [&](SensorScreenRenderer*& currentGLContextScreen){
                    render(currentGLContextScreen, doDoneGLContextCurrent); },
                [&](){ finalizeRendering(); });
        });

    for(auto& screen : screens){
        screen->moveRenderingBufferToThread(sharedScene->renderingThread);
    }
}


// For SCREEN_THREAD_MODE
void SensorScreenRenderer::startRenderingThread()
{
    // This may be unnecessary
    std::unique_lock<std::mutex> lock(scene->renderingMutex);
        
    scene->renderingThread.start([&](){
            scene->concurrentRenderingLoop(
                [&](SensorScreenRenderer*& currentGLContextScreen){
                    render(currentGLContextScreen); },
                [&](){ finalizeRendering(); });
        });

    moveRenderingBufferToThread(scene->renderingThread);
}
    

void SensorScreenRenderer::moveRenderingBufferToThread(QThread& thread)
{
#if USE_QT5_OPENGL
    glContext->moveToThread(&thread);
#endif
}


void SensorRenderer::moveRenderingBufferToMainThread()
{
    for(auto& screen : screens){
        screen->moveRenderingBufferToMainThread();
    }
}


void SensorScreenRenderer::moveRenderingBufferToMainThread()
{
#if USE_QT5_OPENGL
    QThread* mainThread = QApplication::instance()->thread();
    glContext->moveToThread(mainThread);
#endif
}


void SensorScreenRenderer::makeGLContextCurrent()
{
#if USE_QT5_OPENGL
    glContext->makeCurrent(offscreenSurface);
#else
    renderingBuffer->makeCurrent();
#endif
}


void SensorScreenRenderer::doneGLContextCurrent()
{
#if USE_QT5_OPENGL
    glContext->doneCurrent();
#else
    renderingBuffer->doneCurrent();
#endif
}


void GLVisionSimulatorItemImpl::onPreDynamics()
{
    currentTime = simulatorItem->currentTime();

    std::mutex* pQueueMutex = nullptr;
    
    for(size_t i=0; i < sensorRenderers.size(); ++i){
        auto& renderer = sensorRenderers[i];
        if(renderer->elapsedTime >= renderer->cycleTime){
            if(!renderer->isRendering){
                renderer->onsetTime = currentTime;
                renderer->isRendering = true;
                if(useThreadsForSensors){
                    renderer->startConcurrentRendering();
                } else {
                    if(!pQueueMutex){
                        pQueueMutex = &queueMutex;
                        pQueueMutex->lock();
                    }
                    renderer->updateSensorScene(true);
                    sensorQueue.push(renderer);
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
    SensorRenderer* renderer = nullptr;
    SensorScreenRenderer* currentGLContextScreen = nullptr;
    
    while(true){
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            while(true){
                if(isQueueRenderingTerminationRequested){
                    goto exitRenderingQueueLoop;
                }
                if(!sensorQueue.empty()){
                    renderer = sensorQueue.front();
                    sensorQueue.pop();
                    break;
                }
                queueCondition.wait(lock);
            }
        }
        renderer->render(currentGLContextScreen, true);
        
        {
            std::lock_guard<std::mutex> lock(queueMutex);
            renderer->sharedScene->isRenderingFinished = true;
        }
        queueCondition.notify_all();
    }
    
exitRenderingQueueLoop:

    for(size_t i=0; i < sensorRenderers.size(); ++i){
        sensorRenderers[i]->moveRenderingBufferToMainThread();
    }
}


void SensorRenderer::updateSensorScene(bool updateSensorForRenderingThread)
{
    for(auto& scene : scenes){
        scene->updateScene(simImpl->currentTime);
    }
    if(updateSensorForRenderingThread){
        deviceForRendering->copyStateFrom(*device);
    }
}
    

void SensorScene::updateScene(double currentTime)
{
    for(auto& sceneBody : sceneBodies){
        sceneBody->updateLinkPositions();
        sceneBody->updateSceneDevices(currentTime);
    }
}


void SensorRenderer::startConcurrentRendering()
{
    isRendering = true;
    updateSensorScene(true);

    for(auto& scene : scenes){
        scene->startConcurrentRendering();
    }
}


void SensorScene::startConcurrentRendering()
{
    {
        std::lock_guard<std::mutex> lock(renderingMutex);
        isRenderingRequested = true;
    }
    renderingCondition.notify_all();
}


void SensorScene::concurrentRenderingLoop(std::function<void(SensorScreenRenderer*&)> render, std::function<void()> finalizeRendering)
{
    SensorScreenRenderer* currentGLContextScreen = nullptr;
    
    while(true){
        {
            std::unique_lock<std::mutex> lock(renderingMutex);
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

        render(currentGLContextScreen);
        
        {
            std::lock_guard<std::mutex> lock(renderingMutex);
            isRenderingFinished = true;
        }
        renderingCondition.notify_all();
    }
    
exitConcurrentRenderingLoop:
    finalizeRendering();
}


void SensorRenderer::render(SensorScreenRenderer*& currentGLContextScreen, bool doDoneGLContextCurrent)
{
    for(auto& screen : screens){
        screen->render(currentGLContextScreen);
        if(doDoneGLContextCurrent){
            screen->doneGLContextCurrent();
            currentGLContextScreen = nullptr;
        }
    }
}


void SensorScreenRenderer::render(SensorScreenRenderer*& currentGLContextScreen)
{
    if(this != currentGLContextScreen){
        makeGLContextCurrent();
        currentGLContextScreen = this;
    }
    renderer->render();
    renderer->flush();
    storeResultToTmpDataBuffer();
}


void SensorRenderer::finalizeRendering()
{
    for(auto& screen : screens){
        screen->finalizeRendering();
    }
}


void SensorScreenRenderer::finalizeRendering()
{
    doneGLContextCurrent();
    moveRenderingBufferToMainThread();
}


void SensorScreenRenderer::storeResultToTmpDataBuffer()
{
    if(cameraForRendering){
        if(!tmpImage){
            tmpImage = std::make_shared<Image>();
        }
        if(rangeCameraForRendering){
            tmpPoints = std::make_shared<vector<Vector3f>>();
            hasUpdatedData = getRangeCameraData(*tmpImage, *tmpPoints);
        } else {
            hasUpdatedData = getCameraImage(*tmpImage);
        }
    } else if(rangeSensorForRendering){
        tmpRangeData =  std::make_shared<vector<double>>();
        hasUpdatedData = getRangeSensorData(*tmpRangeData);
    }
}


void GLVisionSimulatorItemImpl::onPostDynamics()
{
    if(useThreadsForSensors){
        getVisionDataInThreadsForSensors();
    } else {
        getVisionDataInQueueThread();
    }
}


void GLVisionSimulatorItemImpl::getVisionDataInThreadsForSensors()
{
    auto iter = renderersInRendering.begin();
    while(iter != renderersInRendering.end()){
        auto renderer = *iter;
        if(renderer->elapsedTime >= renderer->latency){
            if(renderer->waitForRenderingToFinish()){
                renderer->copyVisionData();
                renderer->isRendering = false;
            }
        }
        if(renderer->isRendering){
            ++iter;
        } else {
            iter = renderersInRendering.erase(iter);
        }
    }
}


bool SensorRenderer::waitForRenderingToFinish()
{
    for(auto& scene : scenes){
        std::unique_lock<std::mutex> lock(scene->renderingMutex);
        if(!scene->isRenderingFinished){
            if(simImpl->isBestEffortMode){
                if(elapsedTime > cycleTime){
                    elapsedTime = cycleTime;
                }
                return false;
            } else {
                while(!scene->isRenderingFinished){
                    scene->renderingCondition.wait(lock);
                }
            }
        }
    }

    for(auto& scene : scenes){
        scene->isRenderingFinished = false;
    }

    return true;
}
        

void GLVisionSimulatorItemImpl::getVisionDataInQueueThread()
{
    std::unique_lock<std::mutex> lock(queueMutex);
    
    vector<SensorRenderer*>::iterator p = renderersInRendering.begin();
    while(p != renderersInRendering.end()){
        SensorRenderer* renderer = *p;
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


bool SensorRenderer::waitForRenderingToFinish(std::unique_lock<std::mutex>& lock)
{
    if(!sharedScene->isRenderingFinished){
        if(simImpl->isBestEffortMode){
            if(elapsedTime > cycleTime){
                elapsedTime = cycleTime;
            }
            return false;
        } else {
            while(!sharedScene->isRenderingFinished){
                simImpl->queueCondition.wait(lock);
            }
        }
    }
    sharedScene->isRenderingFinished = false;

    return true;
}
        

void SensorRenderer::copyVisionData()
{
    bool hasUpdatedData = true;
    for(auto& screen : screens){
        hasUpdatedData = hasUpdatedData && screen->hasUpdatedData;
    }

    if(hasUpdatedData){
        double delay = simImpl->currentTime - onsetTime;
        if(camera){
            if(camera->lensType()==Camera::NORMAL){
                auto& screen = screens[0];
                if(!screen->tmpImage->empty()){
                    camera->setImage(screen->tmpImage);
                }
                if(rangeCamera){
                    rangeCamera->setPoints(screen->tmpPoints);
                }
            }else{
                std::shared_ptr<Image> image = std::make_shared<Image>();
                if(simImpl->enableInterpolation){
                    fisheyeLensConverter.convertImageInterpolation(image.get());
                }else{
                    fisheyeLensConverter.convertImage(image.get());
                }
                camera->setImage( image );
            }
            camera->setDelay(delay);
        } else if(rangeSensor){
            if(screens.empty()){
                rangeData = std::make_shared<vector<double>>();
            } else if(screens.size() == 1){
                rangeData = screens[0]->tmpRangeData;
            } else {
                rangeData = std::make_shared<vector<double>>();
                vector<double>::iterator src[4];
                int size = 0;
                for(size_t i=0; i < screens.size(); ++i){
                    auto& screen = screens[i];
                    vector<double>& tmpRangeData_ = *screen->tmpRangeData;
                    size += tmpRangeData_.size();
                    src[i] = tmpRangeData_.begin();
                }
                (*rangeData).resize(size);

                vector<double>::iterator dest = (*rangeData).begin();
                const int numPitchSamples = screens[0]->rangeSensorForRendering->numPitchSamples();
                for(int i=0; i < numPitchSamples; ++i){
                    for(size_t j=0; j < screens.size(); ++j){
                        auto screen = screens[j];
                        int n = screen->numUniqueYawSamples;
                        copy(src[j], src[j] + n, dest);
                        advance(src[j], n);
                        advance(dest, n);
                    }
                }
            }
            rangeSensor->setRangeData(rangeData);
            rangeSensor->setDelay(delay);
        }

        if(simImpl->isVisionDataRecordingEnabled){
            device->notifyStateChange();
        } else {
            simBody->notifyUnrecordedDeviceStateChange(device);
        }

        for(auto& screen : screens){
            screen->hasUpdatedData = false;
        }
    }
}


void FisheyeLensConverter::initialize(int width_, int height_, double fov_,
        int screenWidth_, vector<SensorScreenRendererPtr>& screens_)
{
    width = width_;
    height = height_;
    fov = fov_;
    screenWidth = screenWidth_;
    for(auto& screen : screens_){
        screens.push_back(screen);
    }
    fisheyeLensMap.clear();
    fisheyeLensInterpolationMap.clear();
}


void  FisheyeLensConverter::convertImage(Image* image)
{
    image->setSize(width, height, 3);
    unsigned char* pixels = image->pixels();

    if(fisheyeLensMap.empty()){
        fisheyeLensMap.resize(height);
        for(int i=0; i<height; i++){
            fisheyeLensMap[i].resize(width);
        }

        double height2 = height/2.0;
        double screenWidth2 = screenWidth / 2.0;
        double sw22 = screenWidth2 * screenWidth2;
        double r = fov / height;

        for(int j=0; j<height; j++){
            double y = j - height2 + 0.5;
            for(int i=0; i<width; i++){
                bool picked = false;

                SensorScreenRenderer::ScreenId screenId;
                int ii,jj;
                if(i<height){
                    double x = i - height2 + 0.5;;
                    double l = sqrt(x*x+y*y);

                    if(l<=height2){
                        double tanTheta;
                        if(l==0){
                            tanTheta = 0.0;
                        } else {
                            tanTheta = screenWidth2 / l * tan(l*r);
                        }
                        double xx = x*tanTheta;
                        double yy = y*tanTheta;
                        ii = myNearByInt(xx + screenWidth2-0.5);
                        jj = myNearByInt(yy + screenWidth2-0.5);
                        if(0<=ii && ii<screenWidth && 0<=jj && jj<screenWidth){
                            screenId = SensorScreenRenderer::FRONT;
                            picked = true;
                        }else if(ii >= screenWidth){  //right
                            double xx_ = sw22 / xx;
                            double yy_ = screenWidth2 * yy / xx;
                            int iir = myNearByInt(-xx_ + screenWidth2-0.5);
                            int jjr = myNearByInt(yy_ + screenWidth2-0.5);
                            if( 0 <= jjr && jjr < screenWidth){
                                screenId = SensorScreenRenderer::RIGHT;
                                ii = range(iir, 0, screenWidth);
                                jj = jjr;
                                picked = true;
                            }
                        }else if(ii < 0){    //left
                            double xx_ = sw22 / -xx;
                            double yy_ = screenWidth2 * yy / -xx;
                            int iil = myNearByInt(xx_ +screenWidth2-0.5);
                            int jjl = myNearByInt(yy_ + screenWidth2-0.5);
                            if( 0 <= jjl && jjl < screenWidth){
                                screenId = SensorScreenRenderer::LEFT;
                                ii = range(iil, 0, screenWidth);
                                jj = jjl;
                                picked = true;
                            }
                        }
                        if(!picked && jj >= screenWidth){    //bottom
                            double xx_ = screenWidth2 * xx / yy;
                            double yy_ = sw22 / yy;
                            int iib = myNearByInt(xx_ + screenWidth2-0.5);
                            int jjb = myNearByInt(-yy_ + screenWidth2-0.5);
                            screenId = SensorScreenRenderer::BOTTOM;
                            ii = range(iib, 0, screenWidth);
                            jj = range(jjb, 0, screenWidth);
                            picked = true;
                        }else if(!picked && jj < 0){    //top
                            double xx_ = screenWidth2 * xx / -yy;
                            double yy_ = sw22 / -yy;
                            int iit = myNearByInt(xx_ + screenWidth2-0.5);
                            int jjt = myNearByInt(yy_ + screenWidth2-0.5);
                            screenId = SensorScreenRenderer::TOP;
                            ii = range(iit, 0, screenWidth);
                            jj = range(jjt, 0, screenWidth);
                            picked = true;
                        }
                        if(DEBUG_MESSAGE2 && !picked){
                            cout << "Could not pick it up. " << i << " " << j << endl;
                        }
                    }
                }else{
                    double x = i - height - height2 +0.5;
                    double l = sqrt(x*x+y*y);
                    if(l<=height2){
                        double tanTheta;
                        if(l==0){
                            tanTheta = 0.0;
                        } else {
                            tanTheta = screenWidth2 / l * tan(l*r);
                        }
                        double xx = x*tanTheta;
                        double yy = y*tanTheta;
                        ii = myNearByInt(xx + screenWidth2-0.5);
                        jj = myNearByInt(yy + screenWidth2-0.5);
                        if(0<=ii && ii<screenWidth && 0<=jj && jj<screenWidth){
                            screenId = SensorScreenRenderer::BACK;
                            picked = true;
                        }else if(ii >= screenWidth){
                            double xx_ = sw22 / xx;
                            double yy_ = screenWidth2 * yy / xx;
                            int iir = myNearByInt(-xx_ + screenWidth2-0.5);
                            int jjr = myNearByInt(yy_ + screenWidth2-0.5);
                            if( 0 <= jjr && jjr < screenWidth){
                                screenId = SensorScreenRenderer::LEFT;
                                ii = range(iir, 0, screenWidth);
                                jj = jjr;
                                picked = true;
                            }
                        }else if(ii < 0){
                            double xx_ = sw22 / -xx;
                            double yy_ = screenWidth2 * yy / -xx;
                            int iil = myNearByInt(xx_ +screenWidth2-0.5);
                            int jjl = myNearByInt(yy_ + screenWidth2-0.5);
                            if( 0 <= jjl && jjl < screenWidth){
                                screenId = SensorScreenRenderer::RIGHT;
                                ii = range(iil, 0, screenWidth);
                                jj = jjl;
                                picked = true;
                            }
                        }
                        if(!picked && jj >= screenWidth){
                            double xx_ = screenWidth2 * xx / yy;
                            double yy_ = sw22 / yy;
                            int iib = myNearByInt(-xx_ + screenWidth2-0.5);
                            int jjb = myNearByInt(yy_ + screenWidth2-0.5);
                            screenId = SensorScreenRenderer::BOTTOM;
                            ii = range(iib, 0, screenWidth);
                            jj = range(jjb, 0, screenWidth);
                            picked = true;
                        }else if(!picked && jj < 0){
                            double xx_ = screenWidth2 * xx / -yy;
                            double yy_ = sw22 / -yy;
                            int iit = myNearByInt(-xx_ + screenWidth2-0.5);
                            int jjt = myNearByInt(-yy_ + screenWidth2-0.5);
                            screenId = SensorScreenRenderer::TOP;
                            ii = range(iit, 0, screenWidth);
                            jj = range(jjt, 0, screenWidth);
                            picked = true;
                        }
                        if(DEBUG_MESSAGE2 && !picked){
                            cout << "Could not pick it up. " << i << " " << j << endl;
                        }
                    }
                }

                int i_, j_;
                if(!rotateOutputDirection){
                    i_ = i;
                    j_ = j;
                }else{
                    if(i<height){
                        i_ = j;
                        j_ = height - 1 - i;
                    }else{
                        i_ = height - 1 - j + height;
                        j_ = i - height;
                    }
                }
                unsigned char* pix = &pixels[(i_+j_*width)*3];
                if(picked){
                    unsigned char* tempPixels = screens[screenId]->tmpImage->pixels();
                    unsigned char* tempPix = &tempPixels[(int)((ii + jj * screenWidth) * 3)];
                    pix[0] = tempPix[0];
                    pix[1] = tempPix[1];
                    pix[2] = tempPix[2];
                    fisheyeLensMap[j_][i_].sid = screenId;
                    fisheyeLensMap[j_][i_].ix = ii;
                    fisheyeLensMap[j_][i_].iy = jj;
                }else{
                    pix[0] = pix[1] = pix[2] = 0;
                    fisheyeLensMap[j_][i_].sid = SensorScreenRenderer::NONE;
                }
            }
        }
    }else{
        for(int j=0; j<height; j++){
            for(int i=0; i<width; i++){
                unsigned char* pix = &pixels[(i+j*width)*3];
                ScreenIndex& screenIndex = fisheyeLensMap[j][i];
                if(screenIndex.sid != SensorScreenRenderer::NONE){
                    unsigned char* tempPixels = screens[screenIndex.sid]->tmpImage->pixels();
                    unsigned char* tempPix = &tempPixels[(int)((screenIndex.ix + screenIndex.iy * screenWidth) * 3)];
                    pix[0] = tempPix[0];
                    pix[1] = tempPix[1];
                    pix[2] = tempPix[2];
                }else{
                    pix[0] = pix[1] = pix[2] = 0;
                }
            }
        }
    }
}


void  FisheyeLensConverter::convertImageInterpolation(Image* image)
{
    image->setSize(width, height, 3);
    unsigned char* pixels = image->pixels();

    if(fisheyeLensInterpolationMap.empty()){
        fisheyeLensInterpolationMap.resize(height);
        for(int i=0; i<height; i++){
            fisheyeLensInterpolationMap[i].resize(width);
        }

        double height2 = height/2.0;
        double screenWidth2 = screenWidth / 2.0;
        double sw22 = screenWidth2 * screenWidth2;
        double r = fov / height;

        for(int j=0; j<height; j++){
            double y = j - height2 +0.5;
            for(int i=0; i<width; i++){
                bool picked = false;
                double sx,sy;
                int ii,jj;
                if(i<height){  //front
                    double x = i - height2+0.5;
                    double l = sqrt(x*x+y*y);

                    if(l<=height2){
                        double tanTheta;
                        if(l==0){
                            tanTheta = 0.0;
                        } else {
                            tanTheta = screenWidth2 / l * tan(l*r);
                        }
                        double xx = x*tanTheta;
                        double yy = y*tanTheta;
                        ii = myNearByInt(xx + screenWidth2-0.5);
                        jj = myNearByInt(yy + screenWidth2-0.5);
                        if(0<=ii && ii<screenWidth && 0<=jj && jj<screenWidth){  //center
                            sx = xx + screenWidth2-0.5;
                            sy = yy + screenWidth2-0.5;
                            if(sx<0){
                                if(sy<0){
                                    setCubeCorner(TOP_DL, TOP_DL, LEFT_UR, FRONT_UL);
                                }else if(sy>=screenWidth-1){
                                    setCubeCorner(LEFT_DR, FRONT_DL, BOTTOM_UL, BOTTOM_UL);
                                }else{
                                    setVerticalBorder(SensorScreenRenderer::LEFT,
                                            SensorScreenRenderer::FRONT, sy);
                                }
                            }else if(sx>=screenWidth-1){
                                if(sy<0){
                                    setCubeCorner(TOP_DR, TOP_DR, FRONT_UR, RIGHT_UL);
                                }else if(sy>=screenWidth-1){
                                    setCubeCorner(FRONT_DR, RIGHT_DL, BOTTOM_UR, BOTTOM_UR);
                                }else{
                                    setVerticalBorder(SensorScreenRenderer::FRONT,
                                            SensorScreenRenderer::RIGHT, sy);
                                  }
                            }else{
                                if(sy<0){
                                    setHorizontalBorder(SensorScreenRenderer::TOP,
                                            SensorScreenRenderer::FRONT, sx);
                                }else if(sy>=screenWidth-1){
                                    setHorizontalBorder(SensorScreenRenderer::FRONT,
                                            SensorScreenRenderer::BOTTOM, sx);
                                }else{
                                    setCenter(SensorScreenRenderer::FRONT, sx, sy);
                                }
                            }
                            picked = true;
                        }else if(ii >= screenWidth){  //right
                            double xx_ = sw22 / xx;
                            double yy_ = screenWidth2 * yy / xx;
                            int iir = myNearByInt(-xx_ + screenWidth2-0.5);
                            int jjr = myNearByInt(yy_ + screenWidth2-0.5);
                            if( 0 <= jjr && jjr < screenWidth){
                                sx = -xx_ + screenWidth2-0.5;
                                sy = yy_ + screenWidth2-0.5;
                                if(sx<0){
                                    if(sy<0){
                                        setCubeCorner(TOP_DR, TOP_DR, FRONT_UR, RIGHT_UL);
                                    }else if(sy>=screenWidth-1){
                                        setCubeCorner(FRONT_DR, RIGHT_DL, BOTTOM_UR, BOTTOM_UR);
                                    }else{
                                        setVerticalBorder(SensorScreenRenderer::FRONT,
                                                SensorScreenRenderer::RIGHT, sy);
                                    }
                                }else{
                                    if(sy<0){
                                        screenId[0] = screenId[1] = SensorScreenRenderer::TOP;
                                        screenId[2] = screenId[3] = SensorScreenRenderer::RIGHT;
                                        npx[0] = screenWidth - 1;    npy[0] = screenWidth - 1 - (int)sx;
                                        npx[1] = screenWidth - 1;    npy[1] = npy[0] - 1;
                                        npx[2] = sx;                 npy[2] = 0;
                                        npx[3] = npx[2]+1;           npy[3] = 0;
                                    }else if(sy>=screenWidth-1){
                                        screenId[0] = screenId[1] = SensorScreenRenderer::RIGHT;
                                        screenId[2] = screenId[3] = SensorScreenRenderer::BOTTOM;
                                        npx[0] = sx;                 npy[0] = screenWidth - 1;
                                        npx[1] = npx[0]+1;           npy[1] = screenWidth - 1;
                                        npx[2] = screenWidth - 1;    npy[2] = sx;
                                        npx[3] = screenWidth - 1;    npy[3] = npy[2] + 1;
                                    }else{
                                        setCenter(SensorScreenRenderer::RIGHT, sx, sy);
                                    }
                                }
                                picked = true;
                            }
                        }else if(ii < 0){    //left
                            double xx_ = sw22 / -xx;
                            double yy_ = screenWidth2 * yy / -xx;
                            int iil = myNearByInt(xx_ +screenWidth2-0.5);
                            int jjl = myNearByInt(yy_ + screenWidth2-0.5);
                            if( 0 <= jjl && jjl < screenWidth){
                                sx = xx_ + screenWidth2-0.5;
                                sy = yy_ + screenWidth2-0.5;
                                if(sx>=screenWidth-1){
                                    if(sy<0){
                                        setCubeCorner(TOP_DL, TOP_DL, LEFT_UR, FRONT_UL);
                                    }else if(sy>=screenWidth-1){
                                        setCubeCorner(LEFT_DR, FRONT_DL, BOTTOM_UL, BOTTOM_UL);
                                    }else{
                                        setVerticalBorder(SensorScreenRenderer::LEFT,
                                                SensorScreenRenderer::FRONT, sy);
                                    }
                                }else{
                                    if(sy<0){
                                        screenId[0] = screenId[1] = SensorScreenRenderer::TOP;
                                        screenId[2] = screenId[3] = SensorScreenRenderer::LEFT;
                                        npx[0] = 0;    npy[0] = sx;
                                        npx[1] = 0;    npy[1] = npy[0] + 1;
                                        npx[2] = sx;                 npy[2] = 0;
                                        npx[3] = npx[2]+1;           npy[3] = 0;
                                    }else if(sy>=screenWidth-1){
                                        screenId[0] = screenId[1] = SensorScreenRenderer::LEFT;
                                        screenId[2] = screenId[3] = SensorScreenRenderer::BOTTOM;
                                        npx[0] = sx;                 npy[0] = screenWidth - 1;
                                        npx[1] = npx[0]+1;           npy[1] = screenWidth - 1;
                                        npx[2] = 0;                  npy[2] = screenWidth - 1 - (int)sx;
                                        npx[3] = 0;                  npy[3] = npy[2] - 1;
                                    }else{
                                        setCenter(SensorScreenRenderer::LEFT, sx, sy);
                                    }
                                }
                                picked = true;
                            }
                        }
                        if(!picked && jj >= screenWidth){    //bottom
                            double xx_ = screenWidth2 * xx / yy;
                            double yy_ = sw22 / yy;
                            sx = xx_ + screenWidth2-0.5;
                            sy = -yy_ + screenWidth2-0.5;
                            if(sy<0){
                                if(sx<0){
                                    setCubeCorner(FRONT_DL, FRONT_DL, LEFT_DR, BOTTOM_UL);
                                }else if(sx>=screenWidth-1){
                                    setCubeCorner(FRONT_DR, FRONT_DR, BOTTOM_UR, RIGHT_DL);
                                }else{
                                    setHorizontalBorder(SensorScreenRenderer::FRONT,
                                            SensorScreenRenderer::BOTTOM, sx);
                                }
                            }else{
                                if(sx<0){
                                    screenId[0] = screenId[2] = SensorScreenRenderer::LEFT;
                                    screenId[1] = screenId[3] = SensorScreenRenderer::BOTTOM;
                                    npx[0] = screenWidth - 1 -(int)sy;   npy[0] = screenWidth - 1;
                                    npx[1] = 0;                          npy[1] = sy;
                                    npx[2] = npx[0] - 1;                 npy[2] = screenWidth - 1;
                                    npx[3] = 0;                          npy[3] = npy[1]+1;
                                }else if(sx>=screenWidth-1){
                                    screenId[0] = screenId[2] = SensorScreenRenderer::BOTTOM;
                                    screenId[1] = screenId[3] = SensorScreenRenderer::RIGHT;
                                    npx[0] = screenWidth-1;     npy[0] = sy;
                                    npx[1] = sy;                npy[1] = screenWidth - 1;
                                    npx[2] = screenWidth - 1;   npy[2] = npy[0] + 1;
                                    npx[3] = npx[1] + 1;        npy[3] = screenWidth - 1;
                                }else{
                                    setCenter(SensorScreenRenderer::BOTTOM, sx, sy);
                                }
                            }
                            picked = true;
                        }
                        if(!picked && jj < 0){    //top
                            double xx_ = screenWidth2 * xx / -yy;
                            double yy_ = sw22 / -yy;
                            sx = xx_ + screenWidth2-0.5;
                            sy = yy_ + screenWidth2-0.5;
                            if(sy>=screenWidth-1){
                                if(sx<0){
                                    setCubeCorner(LEFT_UR, TOP_DL, FRONT_UL, FRONT_UL);
                                }else if(sx>=screenWidth-1){
                                    setCubeCorner(TOP_DR, RIGHT_UL, FRONT_UR, FRONT_UR);
                                }else{
                                    setHorizontalBorder(SensorScreenRenderer::TOP,
                                            SensorScreenRenderer::FRONT, sx);
                                }
                            }else{
                                if(sx<0){
                                    screenId[0] = screenId[2] = SensorScreenRenderer::LEFT;
                                    screenId[1] = screenId[3] = SensorScreenRenderer::TOP;
                                    npx[0] = sy;           npy[0] = 0;
                                    npx[1] = 0;            npy[1] = sy;
                                    npx[2] = npx[0] + 1;   npy[2] = 0;
                                    npx[3] = 0;            npy[3] = npy[1] + 1;
                                }else if(sx>=screenWidth-1){
                                    screenId[0] = screenId[2] = SensorScreenRenderer::TOP;
                                    screenId[1] = screenId[3] = SensorScreenRenderer::RIGHT;
                                    npx[0] = screenWidth - 1;            npy[0] = sy;
                                    npx[1] = screenWidth - 1 - (int)sy;  npy[1] = 0;
                                    npx[2] = screenWidth - 1;            npy[2] = npy[0] + 1;
                                    npx[3] = npx[1] - 1;                 npy[3] = 0;
                                }else{
                                    setCenter(SensorScreenRenderer::TOP, sx, sy);
                                }
                            }
                            picked = true;
                        }
                        if(DEBUG_MESSAGE2 && !picked){
                            cout << "Could not pick it up. " << i << " " << j << endl;
                        }
                    }
                }else{  //back
                    double x = i - height - height2 + 0.5;
                    double l = sqrt(x*x+y*y);
                    if(l<=height2){
                        double tanTheta;
                        if(l==0){
                            tanTheta = 0.0;
                        } else {
                            tanTheta = screenWidth2 / l * tan(l*r);
                        }
                        double xx = x*tanTheta;
                        double yy = y*tanTheta;
                        ii = myNearByInt(xx + screenWidth2-0.5);
                        jj = myNearByInt(yy + screenWidth2-0.5);
                        if(0<=ii && ii<screenWidth && 0<=jj && jj<screenWidth){  // center
                            sx = xx + screenWidth2-0.5;
                            sy = yy + screenWidth2-0.5;
                            if(sx<0){
                                if(sy<0){
                                    setCubeCorner(TOP_UR, TOP_UR, RIGHT_UR, BACK_UL);
                                }else if(sy>=screenWidth-1){
                                    setCubeCorner(RIGHT_DR, BACK_DL, BOTTOM_DR, BOTTOM_DR);
                                }else{
                                    setVerticalBorder(SensorScreenRenderer::RIGHT,
                                            SensorScreenRenderer::BACK, sy);
                                }
                            }else if(sx>=screenWidth-1){
                                if(sy<0){
                                    setCubeCorner(TOP_UL, TOP_UL, BACK_UR, LEFT_UL);
                                }else if(sy>=screenWidth-1){
                                    setCubeCorner(BACK_DR, LEFT_DL, BOTTOM_DL, BOTTOM_DL);
                                }else{
                                    setVerticalBorder(SensorScreenRenderer::BACK,
                                            SensorScreenRenderer::LEFT, sy);
                                }
                            }else{
                                if(sy<0){
                                    screenId[0] = screenId[1] = SensorScreenRenderer::TOP;
                                    screenId[2] = screenId[3] = SensorScreenRenderer::BACK;
                                    npx[0] = screenWidth - 1 -(int)sx;    npy[0] = 0;
                                    npx[1] = npx[0] - 1;                  npy[1] = 0;
                                    npx[2] = sx;                          npy[2] = 0;
                                    npx[3] = npx[2] + 1;                  npy[3] = 0;
                                }else if(sy>=screenWidth-1){
                                    screenId[0] = screenId[1] = SensorScreenRenderer::BACK;
                                    screenId[2] = screenId[3] = SensorScreenRenderer::BOTTOM;
                                    npx[0] = sx;                          npy[0] = screenWidth - 1;
                                    npx[1] = npx[0] + 1;                  npy[1] = screenWidth - 1;
                                    npx[2] = screenWidth - 1 -(int)sx;;   npy[2] = screenWidth - 1;
                                    npx[3] = npx[2] - 1;                  npy[3] = screenWidth - 1;
                                }else{
                                    setCenter(SensorScreenRenderer::BACK, sx, sy);
                                }
                            }
                            picked = true;
                        }else if(ii >= screenWidth){  //right
                            double xx_ = sw22 / xx;
                            double yy_ = screenWidth2 * yy / xx;
                            int iir = myNearByInt(-xx_ + screenWidth2-0.5);
                            int jjr = myNearByInt(yy_ + screenWidth2-0.5);
                            if( 0 <= jjr && jjr < screenWidth){
                                sx = -xx_ + screenWidth2-0.5;
                                sy = yy_ + screenWidth2-0.5;
                                if(sx<0){
                                    if(sy<0){
                                        setCubeCorner(TOP_UL, TOP_UL, BACK_UR, LEFT_UL);
                                    }else if(sy>=screenWidth-1){
                                        setCubeCorner(BACK_DR, LEFT_DL, BOTTOM_DL, BOTTOM_DL);
                                    }else{
                                        setVerticalBorder(SensorScreenRenderer::BACK,
                                                SensorScreenRenderer::LEFT, sy);
                                    }
                                }else{
                                    if(sy<0){
                                        screenId[0] = screenId[1] = SensorScreenRenderer::TOP;
                                        screenId[2] = screenId[3] = SensorScreenRenderer::LEFT;
                                        npx[0] = 0;                  npy[0] = sx;
                                        npx[1] = 0;                  npy[1] = npy[0] + 1;
                                        npx[2] = sx;                 npy[2] = 0;
                                        npx[3] = npx[2] + 1;         npy[3] = 0;
                                    }else if(sy>=screenWidth-1){
                                        screenId[0] = screenId[1] = SensorScreenRenderer::LEFT;
                                        screenId[2] = screenId[3] = SensorScreenRenderer::BOTTOM;
                                        npx[0] = sx;                 npy[0] = screenWidth - 1;
                                        npx[1] = npx[0]+1;           npy[1] = screenWidth - 1;
                                        npx[2] = 0;                  npy[2] = screenWidth - 1 - (int)sx;
                                        npx[3] = 0;                  npy[3] = npy[2] - 1;
                                    }else{
                                        setCenter(SensorScreenRenderer::LEFT, sx, sy);
                                    }
                                }
                                picked = true;
                            }
                        }else if(ii < 0){   //left
                            double xx_ = sw22 / -xx;
                            double yy_ = screenWidth2 * yy / -xx;
                            int iil = myNearByInt(xx_ + screenWidth2-0.5);
                            int jjl = myNearByInt(yy_ + screenWidth2-0.5);
                            if( 0 <= jjl && jjl < screenWidth){
                                sx = xx_ + screenWidth2-0.5;
                                sy = yy_ + screenWidth2-0.5;
                                if(sx>=screenWidth-1){
                                    if(sy<0){
                                        setCubeCorner(TOP_UR, TOP_UR, RIGHT_UR, BACK_UL);
                                    }else if(sy>=screenWidth-1){
                                        setCubeCorner(RIGHT_DR, BACK_DL, BOTTOM_DR, BOTTOM_DR);
                                    }else{
                                        setVerticalBorder(SensorScreenRenderer::RIGHT,
                                                SensorScreenRenderer::BACK, sy);
                                    }
                                }else{
                                    if(sy<0){
                                        screenId[0] = screenId[1] = SensorScreenRenderer::TOP;
                                        screenId[2] = screenId[3] = SensorScreenRenderer::RIGHT;
                                        npx[0] = screenWidth - 1;    npy[0] = screenWidth - 1 - (int)sx;
                                        npx[1] = screenWidth - 1;    npy[1] = npy[0] - 1;
                                        npx[2] = sx;                 npy[2] = 0;
                                        npx[3] = npx[2]+1;           npy[3] = 0;
                                    }else if(sy>=screenWidth-1){
                                        screenId[0] = screenId[1] = SensorScreenRenderer::RIGHT;
                                        screenId[2] = screenId[3] = SensorScreenRenderer::BOTTOM;
                                        npx[0] = sx;                 npy[0] = screenWidth - 1;
                                        npx[1] = npx[0]+1;           npy[1] = screenWidth - 1;
                                        npx[2] = screenWidth - 1;    npy[2] = sx;
                                        npx[3] = screenWidth - 1;    npy[3] = npy[2] + 1;
                                    }else{
                                        setCenter(SensorScreenRenderer::RIGHT, sx, sy);
                                    }
                                }
                                picked = true;
                            }
                        }
                        if(!picked && jj >= screenWidth){    //bottom
                            double xx_ = screenWidth2 * xx / yy;
                            double yy_ = sw22 / yy;
                            sx = -xx_ + screenWidth2-0.5;
                            sy = yy_ + screenWidth2-0.5;
                            if(sy>=screenWidth-1){
                                if(sx<0){
                                    setCubeCorner(LEFT_DL, BOTTOM_DL, BACK_DR, BACK_DR);
                                }else if(sx>=screenWidth-1){
                                    setCubeCorner(BOTTOM_DR, RIGHT_DR, BACK_DL, BACK_DL);
                                }else{
                                    screenId[0] = screenId[1] = SensorScreenRenderer::BOTTOM;
                                    screenId[2] = screenId[3] = SensorScreenRenderer::BACK;
                                    npx[0] = sx;                         npy[0] = screenWidth - 1;
                                    npx[1] = npx[0]+1;                   npy[1] = screenWidth - 1;
                                    npx[2] = screenWidth - 1 - (int)sx;  npy[2] = screenWidth - 1;
                                    npx[3] = npx[2] - 1;                 npy[3] = screenWidth - 1;
                                }
                            }else{
                                if(sx<0){
                                    screenId[0] = screenId[2] = SensorScreenRenderer::LEFT;
                                    screenId[1] = screenId[3] = SensorScreenRenderer::BOTTOM;
                                    npx[0] = screenWidth - 1 -(int)sy;   npy[0] = screenWidth - 1;
                                    npx[1] = 0;                          npy[1] = sy;
                                    npx[2] = npx[0] - 1;                 npy[2] = screenWidth - 1;
                                    npx[3] = 0;                          npy[3] = npy[1]+1;
                                }else if(sx>=screenWidth-1){
                                    screenId[0] = screenId[2] = SensorScreenRenderer::BOTTOM;
                                    screenId[1] = screenId[3] = SensorScreenRenderer::RIGHT;
                                    npx[0] = screenWidth-1;     npy[0] = sy;
                                    npx[1] = sy;                npy[1] = screenWidth - 1;
                                    npx[2] = screenWidth - 1;   npy[2] = npy[0] + 1;
                                    npx[3] = npx[1] + 1;        npy[3] = screenWidth - 1;
                                }else{
                                    setCenter(SensorScreenRenderer::BOTTOM, sx, sy);
                                }
                            }
                            picked = true;
                        }else if(!picked && jj < 0){   //top
                            double xx_ = screenWidth2 * xx / -yy;
                            double yy_ = sw22 / -yy;
                            sx = -xx_ + screenWidth2-0.5;
                            sy = -yy_ + screenWidth2-0.5;
                            if(sy<0){
                                if(sx<0){
                                    setCubeCorner(BACK_UR, BACK_UR, LEFT_UL, TOP_UL);
                                }else if(sx>=screenWidth-1){
                                    setCubeCorner(BACK_UL, BACK_UL, TOP_UR, TOP_UR);
                                }else{
                                    screenId[0] = screenId[1] = SensorScreenRenderer::BACK;
                                    screenId[2] = screenId[3] = SensorScreenRenderer::TOP;
                                    npx[0] = screenWidth - 1 - (int)sx;     npy[0] = 0;
                                    npx[1] = npx[0] - 1;                    npy[1] = 0;
                                    npx[2] = sx;                            npy[2] = 0;
                                    npx[3] = npx[2] + 1;                    npy[3] = 0;
                                }
                            }else{
                                if(sx<0){
                                    screenId[0] = screenId[2] = SensorScreenRenderer::LEFT;
                                    screenId[1] = screenId[3] = SensorScreenRenderer::TOP;
                                    npx[0] = sy;           npy[0] = 0;
                                    npx[1] = 0;            npy[1] = sy;
                                    npx[2] = npx[0] + 1;   npy[2] = 0;
                                    npx[3] = 0;            npy[3] = npy[1] + 1;
                                }else if(sx>=screenWidth-1){
                                    screenId[0] = screenId[2] = SensorScreenRenderer::TOP;
                                    screenId[1] = screenId[3] = SensorScreenRenderer::RIGHT;
                                    npx[0] = screenWidth - 1;            npy[0] = sy;
                                    npx[1] = screenWidth - 1 - (int)sy;  npy[1] = 0;
                                    npx[2] = screenWidth - 1;            npy[2] = npy[0] + 1;
                                    npx[3] = npx[1]-1;                   npy[3] = 0;
                                }else{
                                    setCenter(SensorScreenRenderer::TOP, sx, sy);
                                }
                            }
                            picked = true;
                        }
                        if(DEBUG_MESSAGE2 && !picked){
                            cout << "Could not pick it up. " << i << " " << j << endl;
                        }
                    }
                }

                int i_, j_;
                if(!rotateOutputDirection){
                    i_ = i;
                    j_ = j;
                }else{
                    if(i<height){
                        i_ = j;
                        j_ = height - 1 - i;
                    }else{
                        i_ = height - 1 - j + height;
                        j_ = i - height;
                    }
                }
                unsigned char* pix = &pixels[(i_+j_*width)*3];
                ScreenIndex4& map = fisheyeLensInterpolationMap[j_][i_];
                if(picked){
                    double dx, dy;
                    if(sx<0){
                        dx = sx + 1;
                    }else{
                        dx = sx - (int)sx;
                    }
                    if(sy<0){
                        dy = sy + 1;
                    }else{
                        dy = sy - (int)sy;
                    }
                    double bias[4];
                    bias[0] = (1.0-dx)*(1.0-dy);
                    bias[1] = dx*(1.0-dy);
                    bias[2] = (1.0-dx)*dy;
                    bias[3] = dx*dy;
                    double pixd[3] = {0.0,0.0,0.0};
                    for(int k=0; k<4; k++){
                        unsigned char* tempPixels = screens[screenId[k]]->tmpImage->pixels();
                        unsigned char* tempPix = &tempPixels[(int)((npx[k] + npy[k] * screenWidth) * 3)];
                        for(int kk=0; kk<3; kk++){
                            pixd[kk] += bias[k] * tempPix[kk];
                        }
                        map.screenIndex[k].sid = screenId[k];
                        map.screenIndex[k].ix = npx[k];
                        map.screenIndex[k].iy = npy[k];
                        map.bias[k] = bias[k];
                    }
                    for(int kk=0; kk<3; kk++){
                        pix[kk] = myNearByInt(pixd[kk]);
                    }
                }else{
                    pix[0] = pix[1] = pix[2] = 0;
                    map.screenIndex[0].sid = SensorScreenRenderer::NONE;
                }
            }
        }
    }else{
        for(int j=0; j<height; j++){
            for(int i=0; i<width; i++){
                unsigned char* pix = &pixels[(i+j*width)*3];
                ScreenIndex4& map = fisheyeLensInterpolationMap[j][i];
                if(map.screenIndex[0].sid != SensorScreenRenderer::NONE){
                    double pixd[3] = {0.0,0.0,0.0};
                    for(int k=0; k<4; k++){
                        unsigned char* tempPixels = screens[map.screenIndex[k].sid]->tmpImage->pixels();
                        unsigned char* tempPix = &tempPixels[(int)((map.screenIndex[k].ix + map.screenIndex[k].iy * screenWidth) * 3)];
                        for(int kk=0; kk<3; kk++){
                            pixd[kk] += map.bias[k] * tempPix[kk];
                        }
                    }
                    for(int kk=0; kk<3; kk++){
                        pix[kk] = myNearByInt(pixd[kk]);
                    }
                }else{
                    pix[0] = pix[1] = pix[2] = 0;
                }
            }
        }
    }
}


bool SensorScreenRenderer::getCameraImage(Image& image)
{
    if(cameraForRendering->imageType() != Camera::COLOR_IMAGE){
        return false;
    }
    image.setSize(pixelWidth, pixelHeight, 3);
    glReadPixels(0, 0, pixelWidth, pixelHeight, GL_RGB, GL_UNSIGNED_BYTE, image.pixels());
    image.applyVerticalFlip();
    return true;
}


bool SensorScreenRenderer::getRangeCameraData(Image& image, vector<Vector3f>& points)
{
#ifndef _WIN32
    unsigned char* colorBuf = 0;
#else
    vector<unsigned char> colorBuf;
#endif

    unsigned char* pixels = 0;

    const bool extractColors = (cameraForRendering->imageType() == Camera::COLOR_IMAGE);
    if(extractColors){
#ifndef _WIN32
        colorBuf = (unsigned char*)alloca(pixelWidth * pixelHeight * 3 * sizeof(unsigned char));
        glReadPixels(0, 0, pixelWidth, pixelHeight, GL_RGB, GL_UNSIGNED_BYTE, colorBuf);
#else
        colorBuf.resize(pixelWidth * pixelHeight * 3 * sizeof(unsigned char));
        glReadPixels(0, 0, pixelWidth, pixelHeight, GL_RGB, GL_UNSIGNED_BYTE, &colorBuf[0]);
#endif
        if(rangeCameraForRendering->isOrganized()){
            image.setSize(pixelWidth, pixelHeight, 3);
        } else {
            image.setSize(pixelWidth * pixelHeight, 1, 3);
        }
        pixels = image.pixels();
    }

#ifndef _WIN32
    float* depthBuf = (float*)alloca(pixelWidth * pixelHeight * sizeof(float));
    glReadPixels(0, 0, pixelWidth, pixelHeight, GL_DEPTH_COMPONENT, GL_FLOAT, depthBuf);
#else
    vector<float> depthBuf(pixelWidth * pixelHeight * sizeof(float));
    glReadPixels(0, 0, pixelWidth, pixelHeight, GL_DEPTH_COMPONENT, GL_FLOAT, &depthBuf[0]);
#endif
    const Matrix4f Pinv = renderer->projectionMatrix().inverse().cast<float>();
    const float fw = pixelWidth;
    const float fh = pixelHeight;
    const int cx = pixelWidth / 2;
    const int cy = pixelHeight / 2;
    const bool isOrganized = rangeCameraForRendering->isOrganized();
    Vector4f n;
    n[3] = 1.0f;
    points.clear();
    points.reserve(pixelWidth * pixelHeight);
    unsigned char* colorSrc = 0;
    
    for(int y = pixelHeight - 1; y >= 0; --y){
        int srcpos = y * pixelWidth;
        if(extractColors){
#ifndef _WIN32
            colorSrc = colorBuf + y * pixelWidth * 3;
#else
            colorSrc = &colorBuf[0] + y *pixelWidth * 3;
#endif
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
            } else if(isOrganized){
                points.push_back(Vector3f());
                Vector3f& p = points.back();
                if(z <= 0.0f){
                    p.z() = numeric_limits<float>::infinity();
                } else {
                    p.z() = -numeric_limits<float>::infinity();
                }
                if(x == cx){
                    p.x() = 0.0;
                } else {
                    p.x() = (x - cx) * numeric_limits<float>::infinity();
                }
                if(y == cy){
                    p.y() = 0.0;
                } else {
                    p.y() = (y - cy) * numeric_limits<float>::infinity();
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


bool SensorScreenRenderer::getRangeSensorData(vector<double>& rangeData)
{
    const double yawRange = rangeSensorForRendering->yawRange();
    const double yawStep = rangeSensorForRendering->yawStep();
    const double maxTanYawAngle = tan(yawRange / 2.0);

    const double pitchRange = rangeSensorForRendering->pitchRange();
    const int numPitchSamples = rangeSensorForRendering->numPitchSamples();
    const double pitchStep = rangeSensorForRendering->pitchStep();
    const double maxTanPitchAngle = tan(pitchRange / 2.0) / cos(yawRange / 2.0);

    const Matrix4 Pinv = renderer->projectionMatrix().inverse();
    const double Pinv_32 = Pinv(3, 2);
    const double Pinv_33 = Pinv(3, 3);
    const double fw = pixelWidth;
    const double fh = pixelHeight;
    const int wh = pixelWidth * pixelHeight;

#ifndef _WIN32
    float* depthBuf;
    if(wh > 1e6){
        depthBuf = (float*)malloc(pixelWidth * pixelHeight * sizeof(float));
    }else{
        depthBuf = (float*)alloca(pixelWidth * pixelHeight * sizeof(float));
    }
    glReadPixels(0, 0, pixelWidth, pixelHeight, GL_DEPTH_COMPONENT, GL_FLOAT, depthBuf);
#else
    vector<float> depthBuf(pixelWidth * pixelHeight * sizeof(float));
    glReadPixels(0, 0, pixelWidth, pixelHeight, GL_DEPTH_COMPONENT, GL_FLOAT, &depthBuf[0]);
#endif

    rangeData.reserve(numUniqueYawSamples * numPitchSamples);

    for(int pitch=0; pitch < numPitchSamples; ++pitch){
        const double pitchAngle = pitch * pitchStep - pitchRange / 2.0;
        const double cosPitchAngle = cos(pitchAngle);

        for(int yaw=0; yaw < numUniqueYawSamples; ++yaw){
            const double yawAngle = yaw * yawStep - yawRange / 2.0;

            int py;
            if(pitchRange == 0.0){
                py = 0;
            } else {
                const double r = (tan(pitchAngle)/cos(yawAngle) + maxTanPitchAngle) / (maxTanPitchAngle * 2.0);
                py = myNearByInt(r * (fh - 1.0));
            }
            const int srcpos = py * pixelWidth;

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

                if(DEBUG_MESSAGE){
                    const Matrix4 Pinv = renderer->projectionMatrix().inverse();
                    const float fw = pixelWidth;
                    const float fh = pixelHeight;
                    const int cx = pixelWidth / 2;
                    const int cy = pixelHeight / 2;
                    Vector4 n;
                    n[3] = 1.0f;
                    n.x() = 2.0 * px / fw - 1.0;
                    n.y() = 2.0 * py / fh - 1.0;
                    n.z() = 2.0 * depth - 1.0f;
                    const Vector4 o = Pinv * n;
                    const double& ww = o[3];
                    double x_ = o[0] / ww;
                    double y_ = o[1] / ww;
                    double z_ = o[2] / ww;
                    double distance_ = sqrt(x_*x_ + y_*y_ + z_*z_);
                    double pitchAngle_ = asin( y_ / distance_);
                    double yawAngle_ = -asin( x_ / sqrt(x_*x_ + z_*z_) );

                    cout << "pixelX= " << px << "  pixelY= " << py << endl;
                    cout << "pitch= " << degree(pitchAngle_)  << " yaw= " << degree(yawAngle_) << endl;
                    cout << "pitch= " << degree(pitchAngle)  << " yaw= " << degree(yawAngle) << endl;
                    cout << "x= " << x_ << " "
                         << "y= " << y_ << " "
                         << "z= " << z_ << endl;
                    double distance = fabs((z / cosPitchAngle) / cos(yawAngle));
                    double x = distance *  cosPitchAngle * sin(-yawAngle);
                    double y  = distance * sin(pitchAngle);
                    cout << "x= " << x << " "
                         << "y= " << y << " "
                         << "z= " << z  << endl;
                    cout << endl;
                }
            } else {
                rangeData.push_back(std::numeric_limits<double>::infinity());
            }
        }
    }

#ifndef _WIN32
    if(wh > 1e6){
        free(depthBuf);
    }
#endif

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
            std::lock_guard<std::mutex> lock(queueMutex);
            isQueueRenderingTerminationRequested = true;
        }
        queueCondition.notify_all();
        queueThread.wait();
        while(!sensorQueue.empty()){
            sensorQueue.pop();
        }
    }
        
    sensorRenderers.clear();
}


SensorRenderer::~SensorRenderer()
{
    if(simImpl->useThreadsForSensors){
        for(auto& scene : scenes){
            scene->terminate();
        }
    }
    screens.clear();
}


void SensorScene::terminate()
{
    {
        std::lock_guard<std::mutex> lock(renderingMutex);
        isTerminationRequested = true;
    }
    renderingCondition.notify_all();
    renderingThread.wait();
}


SensorScreenRenderer::~SensorScreenRenderer()
{
#if USE_QT5_OPENGL
    if(glContext){
        makeGLContextCurrent();
        frameBuffer->release();
        delete frameBuffer;
        delete glContext;
        delete offscreenSurface;
    }
#else
    if(renderingBuffer){
        makeGLContextCurrent();
        delete renderingBuffer;
    }
#endif
    if(renderer){
        delete renderer;
    }
}
    

void GLVisionSimulatorItem::doPutProperties(PutPropertyFunction& putProperty)
{
    SubSimulatorItem::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}


void GLVisionSimulatorItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Target bodies"), bodyNameListString,
                [&](const string& names){ return updateNames(names, bodyNameListString, bodyNames); });
    putProperty(_("Target sensors"), sensorNameListString,
                [&](const string& names){ return updateNames(names, sensorNameListString, sensorNames); });
    putProperty(_("Max frame rate"), maxFrameRate, changeProperty(maxFrameRate));
    putProperty(_("Max latency [s]"), maxLatency, changeProperty(maxLatency));
    putProperty(_("Record vision data"), isVisionDataRecordingEnabled, changeProperty(isVisionDataRecordingEnabled));
    putProperty(_("Thread mode"), threadMode, [&](int index){ return threadMode.select(index); });
    putProperty(_("Best effort"), isBestEffortModeProperty, changeProperty(isBestEffortModeProperty));
    putProperty(_("All scene objects"), shootAllSceneObjects, changeProperty(shootAllSceneObjects));
    putProperty.min(1.0)(_("Precision ratio of range sensors"),
                         rangeSensorPrecisionRatio, changeProperty(rangeSensorPrecisionRatio));
    putProperty.reset()(_("Depth error"), depthError, changeProperty(depthError));
    putProperty.reset()(_("Head light"), isHeadLightEnabled, changeProperty(isHeadLightEnabled));
    putProperty.reset()(_("Additional lights"), areAdditionalLightsEnabled, changeProperty(areAdditionalLightsEnabled));
    putProperty(_("Rotate fisheye lens output direction"), rotateOutputDirection, changeProperty(rotateOutputDirection));
    putProperty(_("Enable Interpolation"), enableInterpolation, changeProperty(enableInterpolation));
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
    archive.write("threadMode", threadMode.selectedSymbol());
    archive.write("bestEffort", isBestEffortModeProperty);
    archive.write("allSceneObjects", shootAllSceneObjects);
    archive.write("rangeSensorPrecisionRatio", rangeSensorPrecisionRatio);
    archive.write("depthError", depthError);
    archive.write("enableHeadLight", isHeadLightEnabled);    
    archive.write("enableAdditionalLights", areAdditionalLightsEnabled);
    archive.write("rotateOutputDirection", rotateOutputDirection);
    archive.write("enableInterpolation", enableInterpolation);
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
    archive.read("bestEffort", isBestEffortModeProperty);
    archive.read("allSceneObjects", shootAllSceneObjects);
    archive.read("rangeSensorPrecisionRatio", rangeSensorPrecisionRatio);
    archive.read("depthError", depthError);
    archive.read("enableHeadLight", isHeadLightEnabled);
    archive.read("enableAdditionalLights", areAdditionalLightsEnabled);
    archive.read("rotateOutputDirection", rotateOutputDirection);
    archive.read("enableInterpolation", enableInterpolation);

    string symbol;
    if(archive.read("threadMode", symbol)){
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
