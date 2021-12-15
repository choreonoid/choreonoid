/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_GLSL_SCENE_RENDERER_H
#define CNOID_BASE_GLSL_SCENE_RENDERER_H

#include <cnoid/GLSceneRenderer>
#include "exportdecl.h"

namespace cnoid {

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

    virtual PolymorphicSceneNodeFunctionSet* renderingFunctions() override;
    virtual void renderCustomGroup(SgGroup* transform, std::function<void()> traverseFunction) override;
    virtual void renderCustomTransform(SgTransform* transform, std::function<void()> traverseFunction) override;
    virtual void renderNode(SgNode* node) override;

    virtual void addNodeDecoration(SgNode* node, NodeDecorationFunction func, int id) override;
    virtual void clearNodeDecorations(int id) override;

    virtual const Affine3& currentModelTransform() const override;
    virtual const Matrix4& projectionMatrix() const override;
    const Isometry3& viewTransform() const;
    const Matrix4& viewProjectionMatrix() const;
    Matrix4 modelViewMatrix() const;
    Matrix4 modelViewProjectionMatrix() const;
    virtual double projectedPixelSizeRatio(const Vector3& position) const override;

    void pushShaderProgram(ShaderProgram* program);
    void popShaderProgram();

    void renderLights(LightingProgram* program);
    void renderFog(LightingProgram* program);

    void dispatchToTransparentPhase(
        ReferencedPtr object, int id,
        const std::function<void(Referenced* object, const Affine3& modelTransform, int id)>& renderingFunction);

    virtual bool initializeGL() override;
    virtual void flushGL() override;
    virtual void clearGL() override;
    virtual void setDefaultFramebufferObject(unsigned int id) override;
    virtual const std::string& glVendor() const override;
    virtual void setViewport(int x, int y, int width, int height) override;
    virtual void updateViewportInformation(int x, int y, int width, int height) override;

    virtual const Vector3& pickedPoint() const override;
    virtual const SgNodePath& pickedNodePath() const override;
    virtual bool isRenderingPickingImage() const override;
    
    virtual void setLightingMode(LightingMode mode) override;
    virtual LightingMode lightingMode() const override;

    virtual bool isShadowCastingAvailable() const override;
    virtual void setWorldLightShadowEnabled(bool on = true) override;
    virtual void setAdditionalLightShadowEnabled(int index, bool on = true) override;
    virtual void clearAdditionalLightShadows() override;
    virtual void setShadowAntiAliasingEnabled(bool on) override;
    
    virtual void setDefaultSmoothShading(bool on) override;
    virtual SgMaterial* defaultMaterial() override;
    virtual void enableTexture(bool on) override;
    virtual void setDefaultPointSize(double size) override;
    virtual void setDefaultLineWidth(double width) override;
    virtual void setNormalVisualizationEnabled(bool on) override;
    virtual void setNormalVisualizationLength(double length) override;
    virtual void requestToClearResources() override;
    virtual void enableUnusedResourceCheck(bool on) override;
    virtual void setDefaultColor(const Vector3f& color) override;
    virtual void setColor(const Vector3f& color) override;
    virtual void setUpsideDown(bool on) override;
    virtual void setBackFaceCullingMode(int mode) override;
    virtual int backFaceCullingMode() const override;
    virtual void setBoundingBoxRenderingForLightweightRenderingGroupEnabled(bool on) override;

    void setLowMemoryConsumptionMode(bool on);

    virtual void setPickingImageOutputEnabled(bool on) override;
    virtual bool getPickingImage(Image& out_image) override;

    class Impl;

protected:
    virtual void doRender() override;
    virtual bool doPick(int x, int y) override;
    
private:
    Impl* impl;
};

}

#endif
