/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_GL_SCENE_RENDERER_H
#define CNOID_BASE_GL_SCENE_RENDERER_H

#include <cnoid/SceneGraph>
#include <cnoid/SceneRenderer>
#include <boost/function.hpp>
#include "exportdecl.h"

namespace cnoid {

class GLSceneRendererImpl;
class SgCustomGLNode;
class Mapping;
    
class CNOID_EXPORT GLSceneRenderer : public SceneRenderer
{
public:
    GLSceneRenderer();
    GLSceneRenderer(SgGroup* root);
    virtual ~GLSceneRenderer();

    virtual SgGroup* sceneRoot();
    virtual SgGroup* scene();
    virtual void clearScene();

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

    virtual const Affine3& currentModelTransform() const;
    virtual const Affine3& currentCameraPosition() const;
    virtual const Matrix4& projectionMatrix() const;
        
    void getViewFrustum(const SgPerspectiveCamera& camera,
                        double& left, double& right, double& bottom, double& top) const;
    void getViewVolume(const SgOrthographicCamera& camera,
                       double& left, double& right, double& bottom, double& top) const;

    bool initializeGL();

    // The following functions cannot be called bofore calling the initializeGL() function.
    bool setSwapInterval(int interval);
    int getSwapInterval() const;

    /**
       This function does the same things as beginRendering() except that
       actual GL commands are not executed.
       This should only be called when you want to initialize
       the rendering without doing any GL rendering commands.
       For example, you can obtain cameras without rendering, and you can render the scene
       after selecting the current camera.
    */
    virtual void initializeRendering();
        
    virtual SignalProxy<void()> sigRenderingRequest();
        
    virtual void beginRendering();
    virtual void endRendering();
    virtual void render();
    virtual void flush();

    bool pick(int x, int y);
    const Vector3& pickedPoint() const;
    const SgNodePath& pickedNodePath() const;

    const Vector3f& backgroundColor() const;
    void setBackgroundColor(const Vector3f& color);

    virtual SgLight* headLight();
    virtual void setHeadLight(SgLight* light);
    void setHeadLightLightingFromBackEnabled(bool on);

    void setAsDefaultLight(SgLight* light);
    void unsetDefaultLight(SgLight* light);
        
    void enableAdditionalLights(bool on);
        
    enum PolygonMode { FILL_MODE, LINE_MODE, POINT_MODE };
    void setPolygonMode(PolygonMode mode);

    void setDefaultLighting(bool on);
    void setDefaultSmoothShading(bool on);
    SgMaterial* defaultMaterial();
    void setDefaultColor(const Vector4f& color);
    void enableTexture(bool on);
    void setDefaultPointSize(double size);
    void setDefaultLineWidth(double width);

    void setNewDisplayListDoubleRenderingEnabled(bool on);

    void showNormalVectors(double length);

    void requestToClearCache();

    /**
       If this is enabled, OpenGL resources such as display lists, vertex buffer objects
       are checked if they are still used or not, and the unused resources are released
       when finalizeRendering() is called. The default value is true.
    */
    virtual void enableUnusedCacheCheck(bool on);

    virtual void visitGroup(SgGroup* group);
    virtual void visitInvariantGroup(SgInvariantGroup* group);
    virtual void visitTransform(SgTransform* transform);
    virtual void visitUnpickableGroup(SgUnpickableGroup* group);
    virtual void visitShape(SgShape* shape);
    virtual void visitPointSet(SgPointSet* pointSet);        
    virtual void visitLineSet(SgLineSet* lineSet);        
    virtual void visitPreprocessed(SgPreprocessed* preprocessed);
    virtual void visitLight(SgLight* light);
    virtual void visitOverlay(SgOverlay* overlay);
    virtual void visitOutlineGroup(SgOutlineGroup* outline);

    bool isPicking();

    void setColor(const Vector4f& color);
    void enableColorMaterial(bool on);
    void setDiffuseColor(const Vector4f& color);
    void setAmbientColor(const Vector4f& color);
    void setEmissionColor(const Vector4f& color);
    void setSpecularColor(const Vector4f& color);
    void setShininess(float shininess);
    void enableCullFace(bool on);
    void setFrontCCW(bool on);
    void enableLighting(bool on);
    void setLightModelTwoSide(bool on);
    void enableBlend(bool on);
    void enableDepthMask(bool on);
    void setPointSize(float size);
    void setLineWidth(float width);
        
private:
    GLSceneRendererImpl* impl;
    friend class SgCustomGLNode;
};


class CNOID_EXPORT SgCustomGLNode : public SgGroup
{
public:
    typedef boost::function<void(GLSceneRenderer& renderer)> RenderingFunction;

    SgCustomGLNode() { }
    SgCustomGLNode(RenderingFunction f) : renderingFunction(f) { }
    virtual SgObject* clone(SgCloneMap& cloneMap) const;
    virtual void accept(SceneVisitor& visitor);
    virtual void render(GLSceneRenderer& renderer);
    void setRenderingFunction(RenderingFunction f);

protected:
    SgCustomGLNode(const SgCustomGLNode& org, SgCloneMap& cloneMap) : SgGroup(org, cloneMap) { }

private:
    RenderingFunction renderingFunction;
};
typedef ref_ptr<SgCustomGLNode> SgCustomGLNodePtr;
}

#endif
