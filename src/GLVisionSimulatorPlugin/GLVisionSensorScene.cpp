#include "GLVisionSensorScene.h"
#include "GLVisionSensorRenderingScreen.h"

using namespace std;
using namespace cnoid;


GLVisionSensorScene::GLVisionSensorScene()
{
    sceneRoot_ = new SgGroup;
    isRenderingRequested = false;
    isRenderingFinished_ = false;
    isTerminationRequested = false;
}


void GLVisionSensorScene::addBody(Body* body, CloneMap& cloneMap)
{
    auto sceneBody = new SceneBody(body);
    sceneBody->cloneShapes(cloneMap);
    sceneBodies_.push_back(sceneBody);
    sceneRoot_->addChild(sceneBody);
}


void GLVisionSensorScene::updateScene(double currentTime)
{
    for(auto& sceneBody : sceneBodies_){
        sceneBody->updateLinkPositions();
        sceneBody->updateSceneDevices(currentTime);
    }
}


void GLVisionSensorScene::startConcurrentRendering()
{
    {
        std::lock_guard<std::mutex> lock(renderingMutex_);
        isRenderingRequested = true;
    }
    renderingCondition_.notify_all();
}


void GLVisionSensorScene::concurrentRenderingLoop
(std::function<void(GLVisionSensorRenderingScreen*&)> render, std::function<void()> finalizeRendering)
{
    GLVisionSensorRenderingScreen* currentGLContextScreen = nullptr;
    
    while(true){
        {
            std::unique_lock<std::mutex> lock(renderingMutex_);
            while(true){
                if(isTerminationRequested){
                    goto exitConcurrentRenderingLoop;
                }
                if(isRenderingRequested){
                    isRenderingRequested = false;
                    break;
                }
                renderingCondition_.wait(lock);
            }
        }

        render(currentGLContextScreen);
        
        {
            std::lock_guard<std::mutex> lock(renderingMutex_);
            isRenderingFinished_ = true;
        }
        renderingCondition_.notify_all();
    }
    
exitConcurrentRenderingLoop:
    finalizeRendering();
}


void GLVisionSensorScene::terminate()
{
    {
        std::lock_guard<std::mutex> lock(renderingMutex_);
        isTerminationRequested = true;
    }
    renderingCondition_.notify_all();
    renderingThread_.wait();
}
