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
    
class CNOID_EXPORT GLSLSceneRenderer : public GLSceneRenderer
{
  public:
    GLSLSceneRenderer();
    GLSLSceneRenderer(SgGroup* root);
    virtual ~GLSLSceneRenderer();

    virtual void setOutputStream(std::ostream& os) override;

    virtual NodeFunctionSet& renderingFunctionSet() override;
    virtual void renderNode(SgNode* node) override;

    virtual const Affine3& currentModelTransform() const override;
    virtual const Matrix4& projectionMatrix() const override;
        
    virtual bool initializeGL() override;
    virtual void flush() override;

    // The following functions cannot be called bofore calling the initializeGL() function.
    bool setSwapInterval(int interval);
    int getSwapInterval() const;

    virtual void render() override;
    virtual bool pick(int x, int y) override;
    virtual const Vector3& pickedPoint() const override;
    virtual const SgNodePath& pickedNodePath() const override;

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

    virtual void requestToClearCache() override;
    virtual void enableUnusedCacheCheck(bool on) override;

    bool isPicking();
    virtual void setColor(const Vector3f& color) override;

    void setDiffuseColor(const Vector3f& color);
    void setAmbientColor(const Vector3f& color);
    void setEmissionColor(const Vector3f& color);
    void setSpecularColor(const Vector3f& color);
    void setShininess(float shininess);
    void setAlpha(float a);
    void setPointSize(float size);
    void setLineWidth(float width);

  protected:
    virtual void onImageUpdated(SgImage* image) override;
    
  private:
    GLSLSceneRendererImpl* impl;
    friend class GLSLSceneRendererImpl;
};

}

#endif
