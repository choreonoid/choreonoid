/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_SCENE_RENDERER_H
#define CNOID_UTIL_SCENE_RENDERER_H

#include <cnoid/SceneGraph>
#include <cnoid/SceneVisitor>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SceneRenderer : public SceneVisitor
{
public:
    virtual ~SceneRenderer() { };

    virtual SgGroup* sceneRoot() = 0;
    virtual void clearScene() = 0;

    virtual int numCameras() const = 0;
    virtual const SgNodePath& cameraPath(int index) const = 0;
    virtual bool getSimplifiedCameraPathStrings(int index, std::vector<std::string>& pathStrings) const = 0;
    virtual SignalProxy<boost::signal<void()> > sigCamerasChanged() const = 0; 
        
    virtual SgCamera* currentCamera() const = 0;
    virtual int currentCameraIndex() const = 0;
    virtual void setCurrentCamera(int index) = 0;
    virtual bool setCurrentCamera(SgCamera* camera) = 0;
    virtual bool setCurrentCamera(std::vector<std::string>& simplifiedPathStrings) = 0;
    virtual SignalProxy< boost::signal<void()> > sigCurrentCameraChanged() = 0;

    virtual void setViewport(int x, int y, int width, int height) = 0;
    virtual Array4i viewport() const = 0;
    virtual double aspectRatio() const = 0; // width / height;

    virtual const Affine3& currentModelMatrix() const = 0;
    // virtual const Affine3& currentModelTransform() const = 0;

    //! This should be removed
    virtual const Affine3& lastViewMatrix() const = 0;
    //virtual const Affine3& currentCameraPosition() const = 0;

    //! This should be removed
    virtual const Matrix4& lastProjectionMatrix() const = 0;
    //virtual const Matrix4& projectionMatrix() const = 0;

    
    virtual void initializeRendering() = 0;

    virtual SignalProxy< boost::signal<void()> > sigRenderingRequest() = 0;
        
    virtual void beginRendering() = 0;
    virtual void endRendering() = 0;
    virtual void render() = 0;
    virtual void flush() = 0;

    virtual SgLight* headLight() = 0;
    virtual void setHeadLight(SgLight* light) = 0;
};

}

#endif
