/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_GL1_SCENE_RENDERER_H
#define CNOID_BASE_GL1_SCENE_RENDERER_H

#include <cnoid/GLSceneRenderer>
#include "exportdecl.h"

namespace cnoid {

class GL1SceneRendererImpl;
    
class CNOID_EXPORT GL1SceneRenderer : public GLSceneRenderer
{
public:
    GL1SceneRenderer(SgGroup* root = nullptr);
    virtual ~GL1SceneRenderer();

    virtual void setOutputStream(std::ostream& os) override;
    virtual NodeFunctionSet* renderingFunctions() override;
    virtual void renderCustomGroup(SgGroup* transform, std::function<void()> traverseFunction) override;
    virtual void renderCustomTransform(SgTransform* transform, std::function<void()> traverseFunction) override;
    virtual void renderNode(SgNode* node) override;
    virtual const Affine3& currentModelTransform() const override;
    virtual const Matrix4& projectionMatrix() const override;
    virtual bool initializeGL() override;
    virtual void flush() override;
    virtual void setViewport(int x, int y, int width, int height) override;

    virtual const Vector3& pickedPoint() const override;
    virtual const SgNodePath& pickedNodePath() const override;
    virtual bool isPicking() const override;
    
    virtual void setLightingMode(int mode) override;
    void setHeadLightLightingFromBackEnabled(bool on);
    virtual void setDefaultSmoothShading(bool on) override;
    virtual SgMaterial* defaultMaterial() override;
    virtual void enableTexture(bool on) override;
    virtual void setDefaultPointSize(double size) override;
    virtual void setDefaultLineWidth(double width) override;

    virtual void showNormalVectors(double length) override;

    virtual void requestToClearResources() override;

    /**
       If this is enabled, OpenGL resources such as display lists, vertex buffer objects
       are checked if they are still used or not, and the unused resources are released
       when finalizeRendering() is called. The default value is true.
    */
    void enableUnusedResourceCheck(bool on);

    virtual void setColor(const Vector3f& color) override;
    virtual void setBackFaceCullingMode(int mode) override;
    virtual int backFaceCullingMode() const override;

  protected:
    virtual void doRender() override;
    virtual bool doPick(int x, int y) override;
    
  private:
    GL1SceneRendererImpl* impl;
    friend class GL1SceneRendererImpl;
};

}

#endif
