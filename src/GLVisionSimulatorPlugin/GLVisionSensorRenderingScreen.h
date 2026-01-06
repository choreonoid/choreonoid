#ifndef CNOID_GL_VISION_SENSOR_PLUGIN_GL_VISION_SENSOR_RENDERING_SCREEN_H
#define CNOID_GL_VISION_SENSOR_PLUGIN_GL_VISION_SENSOR_RENDERING_SCREEN_H

#include "GLVisionSensorScene.h"
#include <cnoid/SceneCameras>

// Define CNOID_ENABLE_EGL based on platform
// In the future, this should be a CMake option
#ifdef __linux__
#define CNOID_ENABLE_EGL
#endif

// Include GLSceneRenderer before Qt OpenGL headers to avoid conflicts.
// This also provides OpenGL type definitions (GLuint, GLenum, etc.) via cnoid/gl.h.
#include <cnoid/GLSceneRenderer>
#ifdef CNOID_ENABLE_EGL
#include <EGL/egl.h>
#include <EGL/eglext.h>
#endif
#include <QOpenGLContext>
#include <QOffscreenSurface>
#include <QOpenGLFramebufferObject>
#include <vector>
#include "exportdecl.h"

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

#ifdef CNOID_ENABLE_EGL
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
