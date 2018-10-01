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
    GLSLSceneRenderer();
    GLSLSceneRenderer(SgGroup* root);
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
    Matrix4 modelViewMatrix() const;
    Matrix4 modelViewProjectionMatrix() const;
    const Matrix4& viewProjectionMatrix() const;

    void pushShaderProgram(ShaderProgram& program, bool isLightingProgram);
    void popShaderProgram();

    void renderLights(LightingProgram* program);
    void renderFog(LightingProgram* program);

    void dispatchToTransparentPhase(std::function<void()> renderingFunction);
        
    virtual bool initializeGL() override;
    virtual void flush() override;

    virtual void setViewport(int x, int y, int width, int height) override;

    virtual const Vector3& pickedPoint() const override;
    virtual const SgNodePath& pickedNodePath() const override;
    virtual bool isPicking() const override;

    virtual void setDefaultLighting(bool on) override;
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

    void setDiffuseColor(const Vector3f& color);
    void setAmbientColor(const Vector3f& color);
    void setEmissionColor(const Vector3f& color);
    void setSpecularColor(const Vector3f& color);
    void setShininess(float shininess);
    void setAlpha(float a);
    void setPointSize(float size);
    void setLineWidth(float width);

    virtual void setBackFaceCullingMode(int mode) override;
    virtual int backFaceCullingMode() const override;

  protected:
    virtual void doRender() override;
    virtual bool doPick(int x, int y) override;
    virtual void onImageUpdated(SgImage* image) override;
    
  private:
    GLSLSceneRendererImpl* impl;
    friend class GLSLSceneRendererImpl;
};

}

#endif
