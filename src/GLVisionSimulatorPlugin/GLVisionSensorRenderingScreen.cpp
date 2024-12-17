#include "GLVisionSensorRenderingScreen.h"
#include "GLVisionSimulatorItem.h"
#include "GLVisionSensorSimulator.h"
#include <cnoid/Camera>
#include <cnoid/GL1SceneRenderer>
#include <cnoid/GLSLSceneRenderer>
#include <cnoid/SceneLights>
#include <cnoid/MathUtil>
#include <QApplication>

using namespace std;
using namespace cnoid;

namespace {

// This does not seem to be necessary
constexpr bool USE_FLUSH_GL_FUNCTION = false;

}


GLVisionSensorRenderingScreen::GLVisionSensorRenderingScreen(GLVisionSensorSimulator* sensorSimulator, int index)
    : sensorSimulator_(sensorSimulator),
      index_(index)
{
    sceneBody = nullptr;
    sceneLink = nullptr;
    sceneDevice = nullptr;
    isLightingEnabled_ = true;
    glContext = nullptr;
    offscreenSurface = nullptr;
    frameBuffer = nullptr;
    sceneRenderer = nullptr;
}


GLVisionSensorRenderingScreen::~GLVisionSensorRenderingScreen()
{
    finalizeGL(true);
}


bool GLVisionSensorRenderingScreen::initialize(GLVisionSensorScenePtr scene, int bodyIndex)
{
    this->scene = scene;

    screenCamera.reset();

    sceneBody = scene->sceneBody(bodyIndex);
    int linkIndex = sensorSimulator_->visionSensor()->link()->index();
    sceneLink = sceneBody->sceneLink(linkIndex);
    if(!sceneLink){
        return false;
    }
    
    sceneDevice = sceneBody->getSceneDevice(sensorSimulator_->visionSensor());

    if(!sensorSimulator_->doInitializeScreenCamera(this)){
        return false;
    }

    if(!initializeGL()){
        return false;
    }

    hasUpdatedData_ = false;

    return true;
}


bool GLVisionSensorRenderingScreen::useCameraDeviceAsScreenCamera()
{
    screenCamera.reset();
    if(sceneDevice){
        if(auto camera = sceneDevice->device<Camera>()){
            screenCamera = sceneDevice->findNodeOfType<SgCamera>();
            if(screenCamera){
                resolutionX_ = camera->resolutionX();
                resolutionY_ = camera->resolutionY();
            }
        }
    }
    return screenCamera != nullptr;
}


void GLVisionSensorRenderingScreen::setScreenCamera
(SgCamera* camera, const Isometry3& T_local, int resolutionX, int resolutionY)
{
    auto cameraPos = new SgPosTransform;
    cameraPos->addChild(camera);

    if(sceneDevice){
        cameraPos->setTransform(T_local);
        sceneDevice->addChild(cameraPos);
    } else {
        auto sensor = sensorSimulator_->visionSensor();
        Isometry3 To;
        To.linear() = sensor->opticalFrameRotation();
        To.translation().setZero();
        cameraPos->setTransform(sensor->T_local() * To * T_local);
        sceneLink->addChild(cameraPos);
    }

    screenCamera = camera;

    resolutionX_ = resolutionX;
    resolutionY_ = resolutionY;
}


void GLVisionSensorRenderingScreen::setScreenCamera
(SgCamera* camera, const Matrix3& R_local, int resolutionX, int resolutionY)
{
    Isometry3 T_local;
    T_local.translation().setZero();
    T_local.linear() = R_local;
    return setScreenCamera(camera, T_local, resolutionX, resolutionY);
}


void GLVisionSensorRenderingScreen::setLightingEnabled(bool on)
{
    isLightingEnabled_ = on;
}


bool GLVisionSensorRenderingScreen::initializeGL()
{
    if(!screenCamera){
        return false;
    }
    
    glContext = new QOpenGLContext;

    QSurfaceFormat format;
    format.setSwapBehavior(QSurfaceFormat::SingleBuffer);
    if(GLSceneRenderer::rendererType() == GLSceneRenderer::GLSL_RENDERER){
        format.setProfile(QSurfaceFormat::CoreProfile);
        format.setVersion(3, 3);
    } else {
        format.setVersion(1, 5);
    }
    glContext->setFormat(format);

    if(!glContext->create()){
        finalizeGL(false);
        return false;
    }
    
    offscreenSurface = new QOffscreenSurface;
    offscreenSurface->setFormat(format);
    offscreenSurface->create();
    if(!offscreenSurface->isValid()){
        finalizeGL(false);
        return false;
    }
        
    bool result = glContext->makeCurrent(offscreenSurface);
    frameBuffer = new QOpenGLFramebufferObject(resolutionX_, resolutionY_, QOpenGLFramebufferObject::CombinedDepthStencil);
    frameBuffer->bind();

    if(!sceneRenderer){
        sceneRenderer = GLSceneRenderer::create();
        sceneRenderer->setFlagVariableToUpdatePreprocessedNodeTree(flagToUpdatePreprocessedNodeTree);
    }

    sceneRenderer->setDefaultFramebufferObject(frameBuffer->handle());

    if(!sceneRenderer->initializeGL()){
        finalizeGL(false);
        return false;
    }

    auto visionSimulatorItem = sensorSimulator_->visionSimulatorItem();
        
    sceneRenderer->setViewport(0, 0, resolutionX_, resolutionY_);
    sceneRenderer->sceneRoot()->addChild(scene->sceneRoot());
    flagToUpdatePreprocessedNodeTree = true;
    sceneRenderer->extractPreprocessedNodes();
    sceneRenderer->setCurrentCamera(screenCamera);
    sceneRenderer->setBackgroundColor(visionSimulatorItem->backgroundColor());

    if(isLightingEnabled_){
        sceneRenderer->headLight()->on(visionSimulatorItem->isHeadLightEnabled());
        sceneRenderer->worldLight()->on(visionSimulatorItem->isWorldLightEnabled());
        sceneRenderer->enableAdditionalLights(visionSimulatorItem->isAdditionalLightSetEnabled());
    } else {
        sceneRenderer->setLightingMode(GLSceneRenderer::NoLighting);
    }

    doneGLContextCurrent();

    return true;
}


void GLVisionSensorRenderingScreen::finalizeGL(bool doMakeCurrent)
{
    if(glContext){
        if(doMakeCurrent){
            makeGLContextCurrent();
        }
        if(sceneRenderer){
            delete sceneRenderer;
            sceneRenderer = nullptr;
        }
        if(frameBuffer){
            frameBuffer->release();
            delete frameBuffer;
            frameBuffer = nullptr;
        }
        if(offscreenSurface){
            delete offscreenSurface;
            offscreenSurface = nullptr;
        }
        delete glContext;
        glContext = nullptr;
    }
}


// For SCREEN_THREAD_MODE
void GLVisionSensorRenderingScreen::startRenderingThread()
{
    // This may be unnecessary
    std::unique_lock<std::mutex> lock(scene->renderingMutex());
        
    scene->renderingThread().start([this](){
            scene->concurrentRenderingLoop(
                [this](GLVisionSensorRenderingScreen*& currentGLContextScreen){
                    render(currentGLContextScreen); },
                [this](){ finalizeRendering(); });
        });

    moveRenderingBufferToThread(scene->renderingThread());
}
    

void GLVisionSensorRenderingScreen::moveRenderingBufferToThread(QThread& thread)
{
    glContext->moveToThread(&thread);
}


void GLVisionSensorRenderingScreen::moveRenderingBufferToMainThread()
{
    QThread* mainThread = QApplication::instance()->thread();
    glContext->moveToThread(mainThread);
}


void GLVisionSensorRenderingScreen::makeGLContextCurrent()
{
    glContext->makeCurrent(offscreenSurface);
}


void GLVisionSensorRenderingScreen::doneGLContextCurrent()
{
    glContext->doneCurrent();
}


void GLVisionSensorRenderingScreen::render(GLVisionSensorRenderingScreen*& currentGLContextScreen)
{
    hasUpdatedData_ = false;
    
    if(this != currentGLContextScreen){
        makeGLContextCurrent();
        currentGLContextScreen = this;
    }
    sceneRenderer->render();

    if(USE_FLUSH_GL_FUNCTION){
        sceneRenderer->flushGL();
    }
    
    sensorSimulator_->doStoreScreenImage(this);
    hasUpdatedData_ = true;
}


void GLVisionSensorRenderingScreen::finalizeRendering()
{
    doneGLContextCurrent();
    moveRenderingBufferToMainThread();
}


void GLVisionSensorRenderingScreen::readImageBuffer(unsigned char* pixels)
{
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, resolutionX_, resolutionY_, GL_RGB, GL_UNSIGNED_BYTE, pixels);
}


void GLVisionSensorRenderingScreen::readImageBuffer(Image& image)
{
    image.setSize(resolutionX_, resolutionY_, 3);
    readImageBuffer(image.pixels());
    image.applyVerticalFlip();
}


void GLVisionSensorRenderingScreen::readDepthBuffer(std::vector<float>& depthBuf)
{
    depthBuf.resize(resolutionX_ * resolutionY_);
    glReadPixels(0, 0, resolutionX_, resolutionY_, GL_DEPTH_COMPONENT, GL_FLOAT, &depthBuf[0]);
}
