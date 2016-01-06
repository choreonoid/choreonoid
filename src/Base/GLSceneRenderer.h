/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_GL_SCENE_RENDERER_H
#define CNOID_BASE_GL_SCENE_RENDERER_H

#include <cnoid/SceneGraph>
#include <cnoid/SceneRenderer>
#include "exportdecl.h"

namespace cnoid {

class GLSceneRendererImpl;
    
class CNOID_EXPORT GLSceneRenderer : public SceneRenderer
{
public:
    GLSceneRenderer();
    GLSceneRenderer(SgGroup* root);
    virtual ~GLSceneRenderer();

    virtual SgGroup* sceneRoot();
    virtual SgGroup* scene();
    virtual void clearScene();

    SignalProxy<void()> sigRenderingRequest();

    virtual int numCameras() const;
    virtual SgCamera* camera(int index);
    virtual const SgNodePath& cameraPath(int index) const;
    virtual SignalProxy<void()> sigCamerasChanged() const; 
        
    virtual SgCamera* currentCamera() const;
    virtual int currentCameraIndex() const;
    virtual void setCurrentCamera(int index);
    virtual bool setCurrentCamera(SgCamera* camera);
    virtual SignalProxy<void()> sigCurrentCameraChanged();

    virtual void setViewport(int x, int y, int width, int height);
    virtual Array4i viewport() const;
    void getViewport(int& out_x, int& out_y, int& out_width, int& out_height) const;
    virtual double aspectRatio() const; // width / height;

    const Vector3f& backgroundColor() const;
    void setBackgroundColor(const Vector3f& color);

    virtual SgLight* headLight();
    virtual void setHeadLight(SgLight* light);
    void setHeadLightLightingFromBackEnabled(bool on);

    void setAsDefaultLight(SgLight* light);
    void unsetDefaultLight(SgLight* light);
        
    void enableAdditionalLights(bool on);

    virtual void flush();

  protected:
    virtual bool initializeGL() = 0;
    void extractPreproNodes();
    virtual void onImageUpdated(SgImage* image) = 0;

    void getViewFrustum(const SgPerspectiveCamera& camera, double& left, double& right, double& bottom, double& top) const;
    void getViewVolume(const SgOrthographicCamera& camera, float& out_left, float& out_right, float& out_bottom, float& out_top) const;    
        
private:
    GLSceneRendererImpl* impl;
    friend class GLSceneRendererImpl;
};

}

#endif
