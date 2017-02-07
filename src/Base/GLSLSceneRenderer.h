/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_GLSL_SCENE_RENDERER_H
#define CNOID_BASE_GLSL_SCENE_RENDERER_H

#include <cnoid/GLSceneRenderer>
#include "exportdecl.h"

//#define CNOID_USE_DISPATCH_CASTING

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

#ifdef CNOID_USE_DISPATCH_CASTING
    void renderGroup(SgGroup* group);
    void renderTransform(SgTransform* transform);
    void renderUnpickableGroup(SgUnpickableGroup* group);
    void renderShape(SgShape* shape);
    void renderPointSet(SgPointSet* pointSet);        
    void renderLineSet(SgLineSet* lineSet);        
    void renderOverlay(SgOverlay* overlay);
    void renderOutlineGroup(SgOutlineGroup* outline);
#else
    void renderGroup(SgNode* group);
    void renderTransform(SgNode* transform);
    void renderUnpickableGroup(SgNode* group);
    void renderShape(SgNode* shape);
    void renderPointSet(SgNode* pointSet);        
    void renderLineSet(SgNode* lineSet);        
    void renderOverlay(SgNode* overlay);
    void renderOutlineGroup(SgNode* outline);
#endif

    bool isPicking();
    virtual void setColor(const Vector3f& color);

    void setDiffuseColor(const Vector3f& color);
    void setAmbientColor(const Vector3f& color);
    void setEmissionColor(const Vector3f& color);
    void setSpecularColor(const Vector3f& color);
    void setShininess(float shininess);
    void setAlpha(float a);
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
