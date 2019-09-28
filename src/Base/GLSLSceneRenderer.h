/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_GLSL_SCENE_RENDERER_H
#define CNOID_BASE_GLSL_SCENE_RENDERER_H

#include <cnoid/GLSceneRenderer>
#include "exportdecl.h"

namespace cnoid {

class GLSLSceneRendererImpl;
class ShaderProgram;
class LightingProgram;
    
class CNOID_EXPORT GLSLSceneRenderer : public GLSceneRenderer
{
  public:
    GLSLSceneRenderer(SgGroup* root = nullptr);
    virtual ~GLSLSceneRenderer();

    static void addExtension(std::function<void(GLSLSceneRenderer* renderer)> func);
    virtual void applyExtensions() override;
    virtual bool applyNewExtensions() override;

    virtual void setOutputStream(std::ostream& os) override;

    virtual NodeFunctionSet* renderingFunctions() override;
    virtual void renderCustomGroup(SgGroup* transform, std::function<void()> traverseFunction) override;
    virtual void renderCustomTransform(SgTransform* transform, std::function<void()> traverseFunction) override;

    virtual void renderNode(SgNode* node) override;

    virtual const Affine3& currentModelTransform() const override;
    virtual const Matrix4& projectionMatrix() const override;
    const Affine3& viewTransform() const;
    const Matrix4& viewProjectionMatrix() const;
    Matrix4 modelViewMatrix() const;
    Matrix4 modelViewProjectionMatrix() const;

    void pushShaderProgram(ShaderProgram& program);
    void popShaderProgram();

    void renderLights(LightingProgram* program);
    void renderFog(LightingProgram* program);

    void dispatchToTransparentPhase(
        Referenced* object, int id,
        std::function<void(Referenced* object, const Affine3& position, int id)> renderingFunction);

    virtual bool initializeGL() override;
    virtual void flush() override;
    virtual void setViewport(int x, int y, int width, int height) override;

    virtual const Vector3& pickedPoint() const override;
    virtual const SgNodePath& pickedNodePath() const override;
    virtual bool isPicking() const override;

    virtual void setLightingMode(int mode) override;
    void setHeadLightLightingFromBackEnabled(bool on);
    virtual void clearShadows() override;
    virtual void enableShadowOfLight(int index, bool on) override;
    virtual void enableShadowAntiAliasing(bool on) override;
    virtual void setDefaultSmoothShading(bool on) override;
    virtual SgMaterial* defaultMaterial() override;
    virtual void enableTexture(bool on) override;
    virtual void setDefaultPointSize(double size) override;
    virtual void setDefaultLineWidth(double width) override;
    virtual void showNormalVectors(double length) override;
    virtual void requestToClearResources() override;
    virtual void enableUnusedResourceCheck(bool on) override;
    virtual void setColor(const Vector3f& color) override;
    virtual void setUpsideDown(bool on) override;
    virtual void setBackFaceCullingMode(int mode) override;
    virtual int backFaceCullingMode() const override;
    virtual void setBoundingBoxRenderingForLightweightRenderingGroupEnabled(bool on) override;

    void setLowMemoryConsumptionMode(bool on);

    virtual void setPickingBufferImageOutputEnabled(bool on) override;
    virtual bool getPickingBufferImage(Image& out_image) override;

    virtual bool isShadowCastingAvailable() const override;

  protected:
    virtual void onSceneGraphUpdated(const SgUpdate& update) override;
    virtual void doRender() override;
    virtual bool doPick(int x, int y) override;
    virtual void onImageUpdated(SgImage* image) override;
    
  private:
    GLSLSceneRendererImpl* impl;
    friend class GLSLSceneRendererImpl;
};

}

#endif
