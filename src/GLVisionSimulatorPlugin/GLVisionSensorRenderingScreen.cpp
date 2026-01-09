#include "GLVisionSensorRenderingScreen.h"
#include "GLVisionSimulatorItem.h"
#include "GLVisionSensorSimulator.h"
#include <cnoid/GLSceneRenderer>
#include <cnoid/gl.h>
#include <cnoid/Camera>
#include <cnoid/SceneLights>
#include <cnoid/MathUtil>
#include <cnoid/MessageOut>
#include <cnoid/NullOut>
#include <cnoid/AppUtil>
#include <cnoid/SceneRendererConfig>
#include <QApplication>
#include <sstream>
#include "gettext.h"

// Define CNOID_ENABLE_EGL based on platform
// In the future, this should be a CMake option
#ifdef __linux__
#define CNOID_ENABLE_EGL
#endif

// Include EGL headers after Qt headers to avoid X11 macro conflicts
#ifdef CNOID_ENABLE_EGL
#include <EGL/egl.h>
#include <EGL/eglext.h>
#endif

using namespace std;
using namespace cnoid;

// EGLInfo structure definition (forward declared in header as nested struct)
#ifdef CNOID_ENABLE_EGL
struct GLVisionSensorRenderingScreen::EGLInfo {
    EGLDisplay display = EGL_NO_DISPLAY;
    EGLContext context = EGL_NO_CONTEXT;
    EGLSurface surface = EGL_NO_SURFACE;
    EGLConfig config = nullptr;
    GLuint fbo = 0;
    GLuint colorRenderbuffer = 0;
    GLuint depthStencilRenderbuffer = 0;
};
#endif

namespace {

// This does not seem to be necessary
constexpr bool USE_FLUSH_GL_FUNCTION = false;

// For loading OpenGL functions via Qt's QOpenGLContext (used by GLSceneRenderer::initializeGL)
static void* getQtProcAddress(const char* name)
{
    return (void*)QOpenGLContext::currentContext()->getProcAddress(name);
}

#ifdef CNOID_ENABLE_EGL
// EGL extension function pointers
static PFNEGLQUERYDEVICESEXTPROC eglQueryDevicesEXT_ = nullptr;
static PFNEGLGETPLATFORMDISPLAYEXTPROC eglGetPlatformDisplayEXT_ = nullptr;

// EGL resources shared across all instances
static bool eglExtensionFunctionsLoaded = false;
static bool eglDeviceEnumerationDone = false;
static bool useEglDeviceEnumeration = false;
static EGLDeviceEXT selectedEglDevice = nullptr;
static EGLDisplay sharedEglDisplay = EGL_NO_DISPLAY;
static int eglInstanceCount = 0;

// EGL OpenGL version determined once for the process
static bool eglOpenGLVersionDetermined = false;
static int eglOpenGLMajorVersion = 0;
static int eglOpenGLMinorVersion = 0;
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
    isMsaaDisabled_ = false;
    glContext = nullptr;
    offscreenSurface = nullptr;
    frameBuffer = nullptr;
    usingEGL = false;
#ifdef CNOID_ENABLE_EGL
    egl = new EGLInfo;
#else
    egl = nullptr;
#endif
    sceneRenderer = nullptr;
}


GLVisionSensorRenderingScreen::~GLVisionSensorRenderingScreen()
{
    finalizeGL(true);
#ifdef CNOID_ENABLE_EGL
    delete egl;
#endif
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


void GLVisionSensorRenderingScreen::disableMsaa()
{
    isMsaaDisabled_ = true;
}


bool GLVisionSensorRenderingScreen::initializeGLX()
{
    glContext = new QOpenGLContext;

    QSurfaceFormat format = QSurfaceFormat::defaultFormat();
    format.setSwapBehavior(QSurfaceFormat::SingleBuffer);
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


#ifdef CNOID_ENABLE_EGL
bool GLVisionSensorRenderingScreen::initializeEGL()
{
    auto mout = MessageOut::master();

    // Track if this is the first EGL initialization
    bool isFirstInitialization = !eglExtensionFunctionsLoaded;

    // Load EGL extension function pointers only once
    if(!eglExtensionFunctionsLoaded) {
        eglQueryDevicesEXT_ =
            (PFNEGLQUERYDEVICESEXTPROC)eglGetProcAddress("eglQueryDevicesEXT");
        eglGetPlatformDisplayEXT_ =
            (PFNEGLGETPLATFORMDISPLAYEXTPROC)eglGetProcAddress("eglGetPlatformDisplayEXT");
        eglExtensionFunctionsLoaded = true;
    }

    // Perform EGL device enumeration only once for the first instance
    if(!eglDeviceEnumerationDone) {
        // Try to use EGL device enumeration for hardware-accelerated rendering
        // This works in headless environments (without X11/Xvfb)
        bool deviceEnumerationSucceeded = false;

        // Check if EGL device enumeration extensions are available
        if(eglQueryDevicesEXT_ && eglGetPlatformDisplayEXT_) {
            // Query available EGL devices
            EGLint numDevices = 0;
            if(eglQueryDevicesEXT_(0, nullptr, &numDevices) && numDevices > 0) {
                std::vector<EGLDeviceEXT> devices(numDevices);
                if(eglQueryDevicesEXT_(numDevices, devices.data(), &numDevices)) {
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
        if(!deviceEnumerationSucceeded && (eglQueryDevicesEXT_ || eglGetPlatformDisplayEXT_)) {
            mout->putWarningln(
                "EGL device enumeration failed. Falling back to EGL_DEFAULT_DISPLAY. "
                "Hardware acceleration may not be available in headless environments.");
        }
    }

    // Create shared EGL display only once for the first instance
    if(sharedEglDisplay == EGL_NO_DISPLAY) {
        if(useEglDeviceEnumeration) {
            // Use device enumeration method
            sharedEglDisplay = eglGetPlatformDisplayEXT_(EGL_PLATFORM_DEVICE_EXT, selectedEglDevice, nullptr);
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
    egl->display = sharedEglDisplay;
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
    if(!eglChooseConfig(egl->display, configAttribs, &egl->config, 1, &numConfigs) || numConfigs == 0) {
        mout->putErrorln("Failed to choose EGL config.");
        return false;
    }

    // Bind OpenGL API
    if(!eglBindAPI(EGL_OPENGL_API)) {
        mout->putErrorln("Failed to bind OpenGL API.");
        return false;
    }

    // Determine the best available OpenGL version once for the process (GLSL mode only)
    if(GLSceneRenderer::rendererType() == GLSceneRenderer::GLSL_RENDERER && !eglOpenGLVersionDetermined) {
        // Try OpenGL versions from 4.6 down to 3.3
        struct { int major; int minor; } versions[] = {
            {4, 6}, {4, 5}, {4, 4}, {4, 3}, {4, 2}, {4, 1}, {4, 0},
            {3, 3}
        };

        for(const auto& v : versions) {
            EGLint testAttribs[] = {
                EGL_CONTEXT_MAJOR_VERSION, v.major,
                EGL_CONTEXT_MINOR_VERSION, v.minor,
                EGL_CONTEXT_OPENGL_PROFILE_MASK, EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT,
                EGL_NONE
            };
            EGLContext testContext = eglCreateContext(egl->display, egl->config, EGL_NO_CONTEXT, testAttribs);
            if(testContext != EGL_NO_CONTEXT) {
                eglDestroyContext(egl->display, testContext);
                eglOpenGLMajorVersion = v.major;
                eglOpenGLMinorVersion = v.minor;
                eglOpenGLVersionDetermined = true;
                break;
            }
        }

        if(!eglOpenGLVersionDetermined) {
            mout->putErrorln(_("OpenGL 3.3 or later is required but not supported."));
            return false;
        }
    }

    // Create EGL context with version based on renderer type
    EGLint contextAttribs[9];
    int attribIndex = 0;
    if(GLSceneRenderer::rendererType() == GLSceneRenderer::GLSL_RENDERER){
        contextAttribs[attribIndex++] = EGL_CONTEXT_MAJOR_VERSION;
        contextAttribs[attribIndex++] = eglOpenGLMajorVersion;
        contextAttribs[attribIndex++] = EGL_CONTEXT_MINOR_VERSION;
        contextAttribs[attribIndex++] = eglOpenGLMinorVersion;
        contextAttribs[attribIndex++] = EGL_CONTEXT_OPENGL_PROFILE_MASK;
        contextAttribs[attribIndex++] = EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT;
    } else {
        contextAttribs[attribIndex++] = EGL_CONTEXT_MAJOR_VERSION;
        contextAttribs[attribIndex++] = 1;
        contextAttribs[attribIndex++] = EGL_CONTEXT_MINOR_VERSION;
        contextAttribs[attribIndex++] = 5;
    }
    contextAttribs[attribIndex] = EGL_NONE;
    egl->context = eglCreateContext(egl->display, egl->config, EGL_NO_CONTEXT, contextAttribs);
    if(egl->context == EGL_NO_CONTEXT) {
        mout->putErrorln("Failed to create EGL context.");
        return false;
    }

    // Create Pbuffer surface
    EGLint pbufferAttribs[] = {
        EGL_WIDTH, resolutionX_,
        EGL_HEIGHT, resolutionY_,
        EGL_NONE
    };
    egl->surface = eglCreatePbufferSurface(egl->display, egl->config, pbufferAttribs);
    if(egl->surface == EGL_NO_SURFACE) {
        mout->putErrorln("Failed to create EGL Pbuffer surface.");
        return false;
    }

    // Make the EGL context current
    if(!eglMakeCurrent(egl->display, egl->surface, egl->surface, egl->context)) {
        mout->putErrorln("Failed to make EGL context current.");
        return false;
    }

    // Output EGL initialization message only on first successful initialization
    if(isFirstInitialization) {
        if(useEglDeviceEnumeration) {
            mout->putln(_("EGL is used for headless rendering with GPU device enumeration."));
        } else {
            mout->putln(_("EGL is used for headless rendering with the default display."));
        }
    }

    return true;
}
#endif


bool GLVisionSensorRenderingScreen::initializeGL()
{
    if(!screenCamera){
        return false;
    }

#ifdef CNOID_ENABLE_EGL
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
#ifdef CNOID_ENABLE_EGL
    else {
        if(!initializeEGL()){
            usingEGL = false;
            finalizeGL(false);
            return false;
        }
    }
#endif

    // Common initialization: create and configure scene renderer
    auto visionSimulatorItem = sensorSimulator_->visionSimulatorItem();

    sceneRenderer = GLSceneRenderer::create();
    sceneRenderer->setFlagVariableToUpdatePreprocessedNodeTree(flagToUpdatePreprocessedNodeTree);

    // Set rendering options from GLVisionSimulatorItem settings
    int effectiveMsaaLevel = 0;
    if(!isMsaaDisabled_){
        int msaaLevel = visionSimulatorItem->msaaLevel();
        effectiveMsaaLevel = (msaaLevel < 0) ? SceneRendererConfig::getSystemDefaultMsaaLevel() : msaaLevel;
    }
    sceneRenderer->setMsaaLevel(effectiveMsaaLevel);
    sceneRenderer->setDepthBufferUpdateEnabled(isDepthBufferUpdateEnabled_);
    sceneRenderer->setInfiniteFarOverrideEnabled(visionSimulatorItem->isInfiniteFarOverrideEnabled());

    // Use a local stringstream to buffer OpenGL info output during initialization
    // to avoid flush() triggering Qt event processing and losing GL context
    std::unique_ptr<std::ostringstream> glInfoBuffer;
    if(sensorSimulator_->isOpenGLInfoOutputEnabled()){
        sceneRenderer->setName(visionSimulatorItem->displayName());
        glInfoBuffer = std::make_unique<std::ostringstream>();
        sceneRenderer->setOutputStream(*glInfoBuffer);
    }

    // Initialize OpenGL functions with the appropriate loader
    bool glInitialized = false;
#ifdef CNOID_ENABLE_EGL
    if(usingEGL){
        glInitialized = sceneRenderer->initializeGL((GLADloadfunc)eglGetProcAddress);
    } else
#endif
    {
        glInitialized = sceneRenderer->initializeGL((GLADloadfunc)getQtProcAddress);
    }

    // Output buffered OpenGL info after initialization is complete
    if(glInfoBuffer){
        sceneRenderer->setOutputStream(cnoid::nullout());
        std::string glInfo = glInfoBuffer->str();
        if(!glInfo.empty()){
            MessageOut::master()->put(glInfo);
        }
    }

    if(!glInitialized){
        finalizeGL(false);
        return false;
    }

    // Create FBO and set it to renderer
#ifdef CNOID_ENABLE_EGL
    if(usingEGL) {
        // Create FBO manually for EGL
        glGenFramebuffers(1, &egl->fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, egl->fbo);

        // Create color renderbuffer
        glGenRenderbuffers(1, &egl->colorRenderbuffer);
        glBindRenderbuffer(GL_RENDERBUFFER, egl->colorRenderbuffer);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_RGB8, resolutionX_, resolutionY_);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, egl->colorRenderbuffer);

        // Create depth-stencil renderbuffer
        // This is needed as the blit destination when MSAA is enabled in GLSLSceneRenderer
        glGenRenderbuffers(1, &egl->depthStencilRenderbuffer);
        glBindRenderbuffer(GL_RENDERBUFFER, egl->depthStencilRenderbuffer);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, resolutionX_, resolutionY_);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, egl->depthStencilRenderbuffer);

        sceneRenderer->setDefaultFramebufferObject(egl->fbo);
    } else
#endif
    {
        sceneRenderer->setDefaultFramebufferObject(frameBuffer->handle());
    }

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
    if(doMakeCurrent){
        makeGLContextCurrent();
    }

    if(sceneRenderer){
        delete sceneRenderer;
        sceneRenderer = nullptr;
    }

#ifdef CNOID_ENABLE_EGL
    if(usingEGL){
        // Clean up per-instance EGL resources
        if(egl->fbo != 0){
            glDeleteFramebuffers(1, &egl->fbo);
            egl->fbo = 0;
        }
        if(egl->colorRenderbuffer != 0){
            glDeleteRenderbuffers(1, &egl->colorRenderbuffer);
            egl->colorRenderbuffer = 0;
        }
        if(egl->depthStencilRenderbuffer != 0){
            glDeleteRenderbuffers(1, &egl->depthStencilRenderbuffer);
            egl->depthStencilRenderbuffer = 0;
        }
        if(egl->context != EGL_NO_CONTEXT){
            eglDestroyContext(egl->display, egl->context);
            egl->context = EGL_NO_CONTEXT;
        }
        if(egl->surface != EGL_NO_SURFACE){
            eglDestroySurface(egl->display, egl->surface);
            egl->surface = EGL_NO_SURFACE;
        }

        // Terminate shared EGL display only when this is the last instance
        eglInstanceCount--;
        if(eglInstanceCount == 0 && sharedEglDisplay != EGL_NO_DISPLAY){
            eglTerminate(sharedEglDisplay);
            sharedEglDisplay = EGL_NO_DISPLAY;
        }

        egl->display = EGL_NO_DISPLAY;
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
#ifdef CNOID_ENABLE_EGL
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
#ifdef CNOID_ENABLE_EGL
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
#ifdef CNOID_ENABLE_EGL
    if(usingEGL){
        eglMakeCurrent(egl->display, egl->surface, egl->surface, egl->context);
    } else {
        glContext->makeCurrent(offscreenSurface);
    }
#else
    glContext->makeCurrent(offscreenSurface);
#endif
}


void GLVisionSensorRenderingScreen::doneGLContextCurrent()
{
#ifdef CNOID_ENABLE_EGL
    if(usingEGL){
        eglMakeCurrent(egl->display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
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
