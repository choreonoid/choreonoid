#include "GLVisionSensorRenderingScreen.h"
#include "GLVisionSimulatorItem.h"
#include "GLVisionSensorSimulator.h"
#include <cnoid/GLSceneRenderer>
#include <cnoid/Camera>
#include <cnoid/SceneLights>
#include <cnoid/MathUtil>
#include <cnoid/MessageOut>
#include <cnoid/Format>
#include <cnoid/AppUtil>
#include <cnoid/SceneRendererConfig>

#ifdef Q_OS_LINUX
// CODEGEN_FUNCPTR is defined in gl_core_3_3.h
#ifndef CODEGEN_FUNCPTR
#define CODEGEN_FUNCPTR
#endif

// Forward declare OpenGL function pointers (from gl_core_3_3.c)
// Note: gl_core_3_3.c uses _ptrc_ prefix for actual variable names
extern "C" {
    // OpenGL loader functions
    enum { ogl_LOAD_FAILED = 0, ogl_LOAD_SUCCEEDED = 1 };
    int ogl_LoadFunctionsEGL();

    // Declare actual function pointer variables with _ptrc_ prefix
    extern void (CODEGEN_FUNCPTR *_ptrc_glGenFramebuffers)(GLsizei n, GLuint * framebuffers);
    extern void (CODEGEN_FUNCPTR *_ptrc_glBindFramebuffer)(GLenum target, GLuint framebuffer);
    extern void (CODEGEN_FUNCPTR *_ptrc_glGenRenderbuffers)(GLsizei n, GLuint * renderbuffers);
    extern void (CODEGEN_FUNCPTR *_ptrc_glBindRenderbuffer)(GLenum target, GLuint renderbuffer);
    extern void (CODEGEN_FUNCPTR *_ptrc_glRenderbufferStorage)(GLenum target, GLenum internalformat, GLsizei width, GLsizei height);
    extern void (CODEGEN_FUNCPTR *_ptrc_glFramebufferRenderbuffer)(GLenum target, GLenum attachment, GLenum renderbuffertarget, GLuint renderbuffer);
    extern GLenum (CODEGEN_FUNCPTR *_ptrc_glCheckFramebufferStatus)(GLenum target);
    extern void (CODEGEN_FUNCPTR *_ptrc_glDeleteFramebuffers)(GLsizei n, const GLuint * framebuffers);
    extern void (CODEGEN_FUNCPTR *_ptrc_glDeleteRenderbuffers)(GLsizei n, const GLuint * renderbuffers);
    extern void (CODEGEN_FUNCPTR *_ptrc_glPixelStorei)(GLenum pname, GLint param);
    extern void (CODEGEN_FUNCPTR *_ptrc_glReadPixels)(GLint x, GLint y, GLsizei width, GLsizei height, GLenum format, GLenum type, void * pixels);
}

// Define macros to match gl_core_3_3.h style
#define glGenFramebuffers _ptrc_glGenFramebuffers
#define glBindFramebuffer _ptrc_glBindFramebuffer
#define glGenRenderbuffers _ptrc_glGenRenderbuffers
#define glBindRenderbuffer _ptrc_glBindRenderbuffer
#define glRenderbufferStorage _ptrc_glRenderbufferStorage
#define glFramebufferRenderbuffer _ptrc_glFramebufferRenderbuffer
#define glCheckFramebufferStatus _ptrc_glCheckFramebufferStatus
#define glDeleteFramebuffers _ptrc_glDeleteFramebuffers
#define glDeleteRenderbuffers _ptrc_glDeleteRenderbuffers
#define glPixelStoreiEGL _ptrc_glPixelStorei
#define glReadPixelsEGL _ptrc_glReadPixels
#endif

#include <QApplication>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

// This does not seem to be necessary
constexpr bool USE_FLUSH_GL_FUNCTION = false;

#ifdef Q_OS_LINUX
// EGL resources shared across all instances
static bool eglDeviceEnumerationDone = false;
static bool useEglDeviceEnumeration = false;
static EGLDeviceEXT selectedEglDevice = nullptr;
static PFNEGLGETPLATFORMDISPLAYEXTPROC eglGetPlatformDisplayEXT_cached = nullptr;
static EGLDisplay sharedEglDisplay = EGL_NO_DISPLAY;
static int eglInstanceCount = 0;
#endif

}


GLVisionSensorRenderingScreen::GLVisionSensorRenderingScreen(GLVisionSensorSimulator* sensorSimulator, int index)
    : sensorSimulator_(sensorSimulator),
      index_(index)
{
    sceneBody = nullptr;
    sceneLink = nullptr;
    sceneDevice = nullptr;
    isLightingEnabled_ = true;
    isDepthBufferUpdateEnabled_ = false;
    glContext = nullptr;
    offscreenSurface = nullptr;
    frameBuffer = nullptr;
    usingEGL = false;

#ifdef Q_OS_LINUX
    // Initialize EGL members
    eglDisplay = nullptr;
    eglContext = nullptr;
    eglSurface = nullptr;
    fbo = 0;
    colorRenderbuffer = 0;
    depthStencilRenderbuffer = 0;
#endif

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


void GLVisionSensorRenderingScreen::setDepthBufferUpdateEnabled(bool on)
{
    isDepthBufferUpdateEnabled_ = on;
}


bool GLVisionSensorRenderingScreen::initializeGLX()
{
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
        return false;
    }

    offscreenSurface = new QOffscreenSurface;
    offscreenSurface->setFormat(format);
    offscreenSurface->create();
    if(!offscreenSurface->isValid()){
        return false;
    }

    glContext->makeCurrent(offscreenSurface);
    frameBuffer = new QOpenGLFramebufferObject(resolutionX_, resolutionY_, QOpenGLFramebufferObject::CombinedDepthStencil);
    frameBuffer->bind();

    return true;
}


#ifdef Q_OS_LINUX
bool GLVisionSensorRenderingScreen::initializeEGL()
{
    auto mout = MessageOut::master();

    // Perform EGL device enumeration only once for the first instance
    if(!eglDeviceEnumerationDone) {
        // Try to use EGL device enumeration for hardware-accelerated rendering
        // This works in headless environments (without X11/Xvfb)
        bool deviceEnumerationSucceeded = false;

        // Get function pointers for EGL device enumeration extensions
        PFNEGLQUERYDEVICESEXTPROC eglQueryDevicesEXT =
            (PFNEGLQUERYDEVICESEXTPROC)eglGetProcAddress("eglQueryDevicesEXT");
        eglGetPlatformDisplayEXT_cached =
            (PFNEGLGETPLATFORMDISPLAYEXTPROC)eglGetProcAddress("eglGetPlatformDisplayEXT");

        if(eglQueryDevicesEXT && eglGetPlatformDisplayEXT_cached) {
            // Query available EGL devices
            EGLint numDevices = 0;
            if(eglQueryDevicesEXT(0, nullptr, &numDevices) && numDevices > 0) {
                std::vector<EGLDeviceEXT> devices(numDevices);
                if(eglQueryDevicesEXT(numDevices, devices.data(), &numDevices)) {
                    // Use the first available device
                    selectedEglDevice = devices[0];
                    deviceEnumerationSucceeded = true;
                }
            }
        }

        // Save the result for subsequent instances
        useEglDeviceEnumeration = deviceEnumerationSucceeded;
        eglDeviceEnumerationDone = true;

        // Output warning only once if enumeration failed
        if(!deviceEnumerationSucceeded && (eglQueryDevicesEXT || eglGetPlatformDisplayEXT_cached)) {
            mout->putWarningln(
                "EGL device enumeration failed. Falling back to EGL_DEFAULT_DISPLAY. "
                "Hardware acceleration may not be available in headless environments.");
        }
    }

    // Create shared EGL display only once for the first instance
    if(sharedEglDisplay == EGL_NO_DISPLAY) {
        if(useEglDeviceEnumeration) {
            // Use device enumeration method
            sharedEglDisplay = eglGetPlatformDisplayEXT_cached(EGL_PLATFORM_DEVICE_EXT, selectedEglDevice, nullptr);
            if(sharedEglDisplay != EGL_NO_DISPLAY) {
                // Initialize EGL with the device display
                EGLint major, minor;
                if(!eglInitialize(sharedEglDisplay, &major, &minor)) {
                    mout->putErrorln("Failed to initialize EGL with enumerated device.");
                    sharedEglDisplay = EGL_NO_DISPLAY;
                    return false;
                }
            } else {
                mout->putErrorln("Failed to get EGL display from enumerated device.");
                return false;
            }
        } else {
            // Fallback to default display method
            sharedEglDisplay = eglGetDisplay(EGL_DEFAULT_DISPLAY);
            if(sharedEglDisplay == EGL_NO_DISPLAY) {
                mout->putErrorln("Failed to get EGL display.");
                return false;
            }

            // Initialize EGL
            EGLint major, minor;
            if(!eglInitialize(sharedEglDisplay, &major, &minor)) {
                mout->putErrorln("Failed to initialize EGL.");
                sharedEglDisplay = EGL_NO_DISPLAY;
                return false;
            }
        }
    }

    // Each instance uses the shared EGL display
    eglDisplay = sharedEglDisplay;
    eglInstanceCount++;

    // Choose EGL configuration
    EGLint configAttribs[] = {
        EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
        EGL_RED_SIZE, 8,
        EGL_GREEN_SIZE, 8,
        EGL_BLUE_SIZE, 8,
        EGL_DEPTH_SIZE, 24,
        EGL_STENCIL_SIZE, 8,
        EGL_NONE
    };
    EGLint numConfigs;
    if(!eglChooseConfig(eglDisplay, configAttribs, &eglConfig, 1, &numConfigs) || numConfigs == 0) {
        mout->putErrorln("Failed to choose EGL config.");
        return false;
    }

    // Bind OpenGL API
    if(!eglBindAPI(EGL_OPENGL_API)) {
        mout->putErrorln("Failed to bind OpenGL API.");
        return false;
    }

    // Create EGL context
    EGLint contextAttribs[] = {
        EGL_CONTEXT_MAJOR_VERSION, 3,
        EGL_CONTEXT_MINOR_VERSION, 3,
        EGL_CONTEXT_OPENGL_PROFILE_MASK, EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT,
        EGL_NONE
    };
    eglContext = eglCreateContext(eglDisplay, eglConfig, EGL_NO_CONTEXT, contextAttribs);
    if(eglContext == EGL_NO_CONTEXT) {
        mout->putErrorln("Failed to create EGL context.");
        return false;
    }

    // Create Pbuffer surface
    EGLint pbufferAttribs[] = {
        EGL_WIDTH, resolutionX_,
        EGL_HEIGHT, resolutionY_,
        EGL_NONE
    };
    eglSurface = eglCreatePbufferSurface(eglDisplay, eglConfig, pbufferAttribs);
    if(eglSurface == EGL_NO_SURFACE) {
        mout->putErrorln("Failed to create EGL Pbuffer surface.");
        return false;
    }

    // Make the EGL context current
    if(!eglMakeCurrent(eglDisplay, eglSurface, eglSurface, eglContext)) {
        mout->putErrorln("Failed to make EGL context current.");
        return false;
    }

    // Load OpenGL functions using EGL
    if(ogl_LoadFunctionsEGL() == ogl_LOAD_FAILED) {
        mout->putErrorln("Failed to load OpenGL functions via EGL.");
        return false;
    }

    // Create FBO manually for EGL
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    // Create color renderbuffer
    glGenRenderbuffers(1, &colorRenderbuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, colorRenderbuffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGB8, resolutionX_, resolutionY_);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, colorRenderbuffer);

    // Create depth-stencil renderbuffer
    // This is needed as the blit destination when MSAA is enabled in GLSLSceneRenderer
    glGenRenderbuffers(1, &depthStencilRenderbuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, depthStencilRenderbuffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, resolutionX_, resolutionY_);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, depthStencilRenderbuffer);

    return true;
}
#endif


bool GLVisionSensorRenderingScreen::initializeGL()
{
    if(!screenCamera){
        return false;
    }

#ifdef Q_OS_LINUX
    // Check if we're in GUI mode or --no-window mode
    if(!AppUtil::isNoWindowMode()) {
        // GUI mode: Use Qt OpenGL (GLX)
        usingEGL = false;
    } else {
        // --no-window mode: Use EGL
        usingEGL = true;
    }
#endif

    // Initialize OpenGL context (GLX or EGL)
    if(!usingEGL) {
        if(!initializeGLX()){
            finalizeGL(false);
            return false;
        }
    }
#ifdef Q_OS_LINUX
    else {
        if(!initializeEGL()){
            usingEGL = false;
            finalizeGL(false);
            return false;
        }
    }
#endif

    // Common initialization: create and configure scene renderer
    if(!sceneRenderer){
        sceneRenderer = GLSceneRenderer::create();
        sceneRenderer->setFlagVariableToUpdatePreprocessedNodeTree(flagToUpdatePreprocessedNodeTree);
    }

#ifdef Q_OS_LINUX
    // Set renderer to use EGL for loading OpenGL functions
    if(usingEGL) {
        sceneRenderer->setUseEGL(true);
        sceneRenderer->setDefaultFramebufferObject(fbo);
    } else
#endif
    {
        sceneRenderer->setDefaultFramebufferObject(frameBuffer->handle());
    }

    // Set MSAA level from GLVisionSimulatorItem settings
    int msaaLevel = sensorSimulator_->visionSimulatorItem()->msaaLevel();
    int effectiveMsaaLevel = (msaaLevel < 0) ? SceneRendererConfig::getSystemDefaultMsaaLevel() : msaaLevel;
    sceneRenderer->setMsaaLevel(effectiveMsaaLevel);
    sceneRenderer->setDepthBufferUpdateEnabled(isDepthBufferUpdateEnabled_);

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

    // Output OpenGL information if enabled for this sensor
    if(sensorSimulator_->isOpenGLInfoOutputEnabled()){
        auto mout = MessageOut::master();
        mout->putln(
            formatR(_("OpenGL {0} (GLSL {1}) is available for {2}."),
                    sceneRenderer->glVersionString(),
                    sceneRenderer->glslVersionString(),
                    sensorSimulator_->visionSimulatorItem()->displayName()));
        mout->putln(
            formatR(_("Driver profile: {0} {1}."),
                    sceneRenderer->glVendorString(), sceneRenderer->glRendererString()));
    }

    doneGLContextCurrent();

    return true;
}


void GLVisionSensorRenderingScreen::finalizeGL(bool doMakeCurrent)
{
    if(doMakeCurrent){
        makeGLContextCurrent();
    }

    if(sceneRenderer){
        delete sceneRenderer;
        sceneRenderer = nullptr;
    }

#ifdef Q_OS_LINUX
    if(usingEGL){
        // Clean up per-instance EGL resources
        if(fbo != 0){
            glDeleteFramebuffers(1, &fbo);
            fbo = 0;
        }
        if(colorRenderbuffer != 0){
            glDeleteRenderbuffers(1, &colorRenderbuffer);
            colorRenderbuffer = 0;
        }
        if(depthStencilRenderbuffer != 0){
            glDeleteRenderbuffers(1, &depthStencilRenderbuffer);
            depthStencilRenderbuffer = 0;
        }
        if(eglContext != EGL_NO_CONTEXT){
            eglDestroyContext(eglDisplay, eglContext);
            eglContext = EGL_NO_CONTEXT;
        }
        if(eglSurface != EGL_NO_SURFACE){
            eglDestroySurface(eglDisplay, eglSurface);
            eglSurface = EGL_NO_SURFACE;
        }

        // Terminate shared EGL display only when this is the last instance
        eglInstanceCount--;
        if(eglInstanceCount == 0 && sharedEglDisplay != EGL_NO_DISPLAY){
            eglTerminate(sharedEglDisplay);
            sharedEglDisplay = EGL_NO_DISPLAY;
        }

        eglDisplay = EGL_NO_DISPLAY;
    } else
#endif
    {
        // Clean up Qt OpenGL resources
        if(frameBuffer){
            frameBuffer->release();
            delete frameBuffer;
            frameBuffer = nullptr;
        }
        if(offscreenSurface){
            delete offscreenSurface;
            offscreenSurface = nullptr;
        }
        if(glContext){
            delete glContext;
            glContext = nullptr;
        }
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
#ifdef Q_OS_LINUX
    if(!usingEGL){
        // Qt OpenGL mode: need to move context to thread
        glContext->moveToThread(&thread);
    }
    // EGL mode: contexts are managed per-thread, no explicit move needed
#else
    glContext->moveToThread(&thread);
#endif
}


void GLVisionSensorRenderingScreen::moveRenderingBufferToMainThread()
{
#ifdef Q_OS_LINUX
    if(!usingEGL){
        // Qt OpenGL mode: need to move context to main thread
        QThread* mainThread = QApplication::instance()->thread();
        glContext->moveToThread(mainThread);
    }
    // EGL mode: contexts are managed per-thread, no explicit move needed
#else
    QThread* mainThread = QApplication::instance()->thread();
    glContext->moveToThread(mainThread);
#endif
}


void GLVisionSensorRenderingScreen::makeGLContextCurrent()
{
#ifdef Q_OS_LINUX
    if(usingEGL){
        eglMakeCurrent(eglDisplay, eglSurface, eglSurface, eglContext);
    } else {
        glContext->makeCurrent(offscreenSurface);
    }
#else
    glContext->makeCurrent(offscreenSurface);
#endif
}


void GLVisionSensorRenderingScreen::doneGLContextCurrent()
{
#ifdef Q_OS_LINUX
    if(usingEGL){
        eglMakeCurrent(eglDisplay, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    } else {
        glContext->doneCurrent();
    }
#else
    glContext->doneCurrent();
#endif
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
#ifdef Q_OS_LINUX
    if(usingEGL){
        glPixelStoreiEGL(GL_PACK_ALIGNMENT, 1);
        glReadPixelsEGL(0, 0, resolutionX_, resolutionY_, GL_RGB, GL_UNSIGNED_BYTE, pixels);
    } else
#endif
    {
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, resolutionX_, resolutionY_, GL_RGB, GL_UNSIGNED_BYTE, pixels);
    }
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
#ifdef Q_OS_LINUX
    if(usingEGL){
        glReadPixelsEGL(0, 0, resolutionX_, resolutionY_, GL_DEPTH_COMPONENT, GL_FLOAT, &depthBuf[0]);
    } else
#endif
    {
        glReadPixels(0, 0, resolutionX_, resolutionY_, GL_DEPTH_COMPONENT, GL_FLOAT, &depthBuf[0]);
    }
}
