/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_SCENE_RENDERER_H
#define CNOID_UTIL_SCENE_RENDERER_H

#include <cnoid/SceneGraph>
#include <cnoid/SceneVisitor>
#include <cnoid/SceneProcessor>
#include "exportdecl.h"

namespace cnoid {

class SceneRendererImpl;

class CNOID_EXPORT SceneRenderer : public SceneVisitor
{
public:
    SceneRenderer();
    virtual ~SceneRenderer();
    
    virtual SgGroup* sceneRoot() = 0;
    virtual SgGroup* scene() = 0;
    virtual void clearScene();

    int numCameras() const;
    SgCamera* camera(int index);
    const SgNodePath& cameraPath(int index) const;
    virtual const Affine3& currentCameraPosition() const;
    SignalProxy<void()> sigCamerasChanged() const;
    SgCamera* currentCamera() const;
    int currentCameraIndex() const;
    void setCurrentCamera(int index);
    bool setCurrentCamera(SgCamera* camera);
    SignalProxy<void()> sigCurrentCameraChanged();
    bool getSimplifiedCameraPathStrings(int index, std::vector<std::string>& out_pathStrings);
    int findCameraPath(const std::vector<std::string>& simplifiedPathStrings);
    bool setCurrentCameraPath(const std::vector<std::string>& simplifiedPathStrings);

    int numLights() const;
    void getLightInfo(int index, SgLight*& out_light, Affine3& out_position) const;
    void setAsDefaultLight(SgLight* light);
    void unsetDefaultLight(SgLight* light);
    SgLight* headLight();
    void setHeadLight(SgLight* light);
    void enableAdditionalLights(bool on);

    void enableFog(bool on);
    bool isFogEnabled() const;
    int numFogs() const;
    SgFog* fog(int index) const;

    virtual void setViewport(int x, int y, int width, int height) = 0;
    virtual Array4i viewport() const = 0;
    virtual double aspectRatio() const = 0; // width / height;

    virtual const Affine3& currentModelTransform() const = 0;
    virtual const Matrix4& projectionMatrix() const = 0;


    /**
       This function updates the information on preprocessed nodes such as
       cameras, lights, and fogs.
    */
    virtual void extractPreprocessedNodes();
    
    virtual void render() = 0;
    virtual void flush() = 0;

    Signal<void()>& sigRenderingRequest();

protected:
    virtual void onSceneGraphUpdated(const SgUpdate& update);


private:
    SceneRendererImpl* impl;
};

}

#endif
