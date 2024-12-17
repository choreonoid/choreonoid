#ifndef CNOID_GL_VISION_SENSOR_PLUGIN_GL_VISION_SENSOR_SCENE_H
#define CNOID_GL_VISION_SENSOR_PLUGIN_GL_VISION_SENSOR_SCENE_H

#include <cnoid/SceneBody>
#include <QThread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <vector>

namespace cnoid {

class GLVisionSensorRenderingScreen;

class GLVisionSensorScene : public Referenced
{
public:
    GLVisionSensorScene();
    void addBody(Body* body, CloneMap& cloneMap);
    void updateScene(double currentTime);
    void startConcurrentRendering();
    void concurrentRenderingLoop(
        std::function<void(GLVisionSensorRenderingScreen*&)> render, std::function<void()> finalizeRendering);
    void terminate();

    SgGroup* sceneRoot() const { return sceneRoot_; }
    SceneBody* sceneBody(int index) const { return sceneBodies_[index]; }

    class QThreadEx : public QThread
    {
        std::function<void()> function;
    public:
        void start(std::function<void()> function){
            this->function = function;
            QThread::start();
        }
        virtual void run() override {
            function();
        }
    };
    
    QThreadEx& renderingThread() { return renderingThread_; }
    std::condition_variable& renderingCondition() { return renderingCondition_; }
    std::mutex& renderingMutex() { return renderingMutex_; }
    bool isRenderingFinished() const { return isRenderingFinished_; }
    void setRenderingFinished(bool on){ isRenderingFinished_ = on; }

private:
    SgGroupPtr sceneRoot_;
    std::vector<SceneBodyPtr> sceneBodies_;
    QThreadEx renderingThread_;
    std::condition_variable renderingCondition_;
    std::mutex renderingMutex_;
    bool isRenderingRequested;
    bool isRenderingFinished_;
    bool isTerminationRequested;
};

typedef ref_ptr<GLVisionSensorScene> GLVisionSensorScenePtr;

}

#endif
