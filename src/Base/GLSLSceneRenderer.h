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

    virtual void setOutputStream(std::ostream& os);

    virtual const Affine3& currentModelTransform() const;
    virtual const Matrix4& projectionMatrix() const;
        
    virtual bool initializeGL();
    virtual void flush();

    // The following functions cannot be called bofore calling the initializeGL() function.
    bool setSwapInterval(int interval);
    int getSwapInterval() const;

    virtual void render();
    virtual bool pick(int x, int y);
    virtual const Vector3& pickedPoint() const;
    virtual const SgNodePath& pickedNodePath() const;

    virtual void setDefaultLighting(bool on);
    void setHeadLightLightingFromBackEnabled(bool on);
    virtual void clearShadows();
    virtual void enableShadowOfLight(int index, bool on);
    virtual void enableShadowAntiAliasing(bool on);
    virtual void setDefaultSmoothShading(bool on);
    virtual SgMaterial* defaultMaterial();
    virtual void enableTexture(bool on);
    virtual void setDefaultPointSize(double size);
    virtual void setDefaultLineWidth(double width);

    virtual void showNormalVectors(double length);

    virtual void requestToClearCache();
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
    virtual void setColor(const Vector3f& color);

    void enableColorMaterial(bool on);
    void setDiffuseColor(const Vector3f& color);
    void setAmbientColor(const Vector3f& color);
    void setEmissionColor(const Vector3f& color);
    void setSpecularColor(const Vector3f& color);
    void setShininess(float shininess);
    void setAlpha(float a);
    void enableCullFace(bool on);
    void setFrontCCW(bool on);
    void enableLighting(bool on);
    void setLightModelTwoSide(bool on);
    void enableBlend(bool on);
    void enableDepthMask(bool on);
    void setPointSize(float size);
    void setLineWidth(float width);

  protected:
    virtual void onImageUpdated(SgImage* image);    
        
  private:
    GLSLSceneRendererImpl* impl;
    friend class GLSLSceneRendererImpl;
};

}

#endif
