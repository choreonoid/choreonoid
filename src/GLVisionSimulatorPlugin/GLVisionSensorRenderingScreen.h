#ifndef CNOID_GL_VISION_SENSOR_PLUGIN_GL_VISION_SENSOR_RENDERING_SCREEN_H
#define CNOID_GL_VISION_SENSOR_PLUGIN_GL_VISION_SENSOR_RENDERING_SCREEN_H

#include "GLVisionSensorScene.h"
#include <cnoid/SceneCameras>
#include <QOpenGLContext>
#include <QOffscreenSurface>
#include <QOpenGLFramebufferObject>
#include <vector>
#include "exportdecl.h"

#ifdef Q_OS_LINUX
// Include EGL headers
#include <EGL/egl.h>
#include <EGL/eglext.h>

// Qt 5.12 (Ubuntu 20.04) has incomplete handling of X11 macro conflicts.
// EGL headers internally include X11 headers which define these names as macros,
// conflicting with Qt's enum values and function names.
// Qt 5.15+ handles this internally, so we only need this workaround for older versions.
#if QT_VERSION < QT_VERSION_CHECK(5, 15, 0)
#undef Status
#undef None
#undef Bool
#undef CursorShape
#undef Expose
#undef KeyPress
#undef KeyRelease
#undef FocusIn
#undef FocusOut
#undef FontChange
#undef Unsorted
#endif

// Forward declare OpenGL types
typedef unsigned int GLuint;
#endif

namespace cnoid {

class GLVisionSensorSimulator;
class GLSceneRenderer;
class Camera;
class Image;

class CNOID_EXPORT GLVisionSensorRenderingScreen : public Referenced
{
public:
    GLVisionSensorRenderingScreen(GLVisionSensorSimulator* sensorSimulator, int index);
    ~GLVisionSensorRenderingScreen();

    bool initialize(GLVisionSensorScenePtr scene, int bodyIndex);
    void startRenderingThread();
    void moveRenderingBufferToThread(QThread& thread);
    void moveRenderingBufferToMainThread();
    void render(GLVisionSensorRenderingScreen*& currentGLContextScreen);
    void doneGLContextCurrent();
    void finalizeRendering();

    GLVisionSensorSimulator* sensorSimulator() { return sensorSimulator_; }
    int index() const { return index_; }
    GLSceneRenderer* renderer() { return sceneRenderer; }
    bool useCameraDeviceAsScreenCamera();
    void setScreenCamera(SgCamera* camera, const Isometry3& T_local, int resolutionX, int resolutionY);
    void setScreenCamera(SgCamera* camera, const Matrix3& R_local, int resolutionX, int resolutionY);
    int resolutionX() const { return resolutionX_; }
    int resolutionY() const { return resolutionY_; }
    void setLightingEnabled(bool on);
    void setDepthBufferUpdateEnabled(bool on);
    void readImageBuffer(unsigned char* pixels);
    void readImageBuffer(Image& image);
    void readDepthBuffer(std::vector<float>& depthBuf);
    bool hasUpdatedData() const { return hasUpdatedData_; }

private:
    GLVisionSensorSimulator* sensorSimulator_;
    int index_;
    GLVisionSensorScenePtr scene;
    SceneBody* sceneBody;
    SceneLink* sceneLink;
    SceneDevice* sceneDevice;
    SgCameraPtr screenCamera;
    bool isLightingEnabled_;
    bool isDepthBufferUpdateEnabled_;
    bool hasUpdatedData_;
    QOpenGLContext* glContext;
    QOffscreenSurface* offscreenSurface;
    QOpenGLFramebufferObject* frameBuffer;
    bool usingEGL;

#ifdef Q_OS_LINUX
    // EGL resources
    EGLDisplay eglDisplay;
    EGLContext eglContext;
    EGLSurface eglSurface;
    EGLConfig eglConfig;
    GLuint fbo;
    GLuint colorRenderbuffer;
    GLuint depthStencilRenderbuffer;
#endif

    GLSceneRenderer* sceneRenderer;
    int resolutionX_;
    int resolutionY_;
    bool flagToUpdatePreprocessedNodeTree;

    bool initializeGL();
    bool initializeGLX();
    bool initializeEGL();
    void finalizeGL(bool doMakeCurrent);
    void makeGLContextCurrent();
};

typedef ref_ptr<GLVisionSensorRenderingScreen> GLVisionSensorRenderingScreenPtr;

}

#endif
