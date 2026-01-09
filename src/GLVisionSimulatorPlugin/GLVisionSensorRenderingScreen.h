#ifndef CNOID_GL_VISION_SENSOR_PLUGIN_GL_VISION_SENSOR_RENDERING_SCREEN_H
#define CNOID_GL_VISION_SENSOR_PLUGIN_GL_VISION_SENSOR_RENDERING_SCREEN_H

#include "GLVisionSensorScene.h"
#include <cnoid/SceneCameras>
#include <cnoid/GLSceneRenderer>
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
    void disableMsaa();
    void readImageBuffer(unsigned char* pixels);
    void readImageBuffer(Image& image);
    void readDepthBuffer(std::vector<float>& depthBuf);
    bool hasUpdatedData() const { return hasUpdatedData_; }

    // Depth buffer helper functions for reversed/standard depth buffer handling
    static float getDepthClearValue(bool isReversedDepth) {
        return isReversedDepth ? 0.0f : 1.0f;
    }
    static bool isValidDepthValue(float depth, bool isReversedDepth) {
        // Reversed: near=1.0, far=0.0, clear=0.0 -> valid range is (0.0, 1.0]
        // Standard: near=0.0, far=1.0, clear=1.0 -> valid range is [0.0, 1.0)
        return isReversedDepth ? (depth > 0.0f && depth <= 1.0f)
                               : (depth >= 0.0f && depth < 1.0f);
    }
    static float depthToNdcZ(float depth, bool isReversedDepth) {
        // Reversed: NDC z-range is [0, 1], no transformation needed
        // Standard: depth [0,1] -> NDC [-1,1]
        return isReversedDepth ? depth : (2.0f * depth - 1.0f);
    }
    static double depthToNdcZ(double depth, bool isReversedDepth) {
        return isReversedDepth ? depth : (2.0 * depth - 1.0);
    }

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
    bool isMsaaDisabled_;
    bool hasUpdatedData_;
    QOpenGLContext* glContext;
    QOffscreenSurface* offscreenSurface;
    QOpenGLFramebufferObject* frameBuffer;
    bool usingEGL;

    struct EGLInfo;
    EGLInfo* egl;

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
