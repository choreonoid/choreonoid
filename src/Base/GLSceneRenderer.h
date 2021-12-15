/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_GL_SCENE_RENDERER_H
#define CNOID_BASE_GL_SCENE_RENDERER_H

#include <cnoid/SceneRenderer>
#include "exportdecl.h"

namespace cnoid {

class Image;
    
class CNOID_EXPORT GLSceneRenderer : public SceneRenderer
{
public:
    static void initializeClass();

    enum RendererType { GL1_RENDERER, GLSL_RENDERER };
    static int rendererType();
    static GLSceneRenderer* create(SgGroup* root = nullptr);
    
    GLSceneRenderer(SgGroup* root = nullptr);
    virtual ~GLSceneRenderer();

    virtual void setOutputStream(std::ostream& os);

    virtual SgGroup* sceneRoot() override;
    virtual SgGroup* scene() override;

    virtual bool initializeGL() = 0;
    virtual void flushGL() = 0;

    /**
       This function clears all the OpenGL resourses used in the renderer.
       The function should be called when the renderer is deleted.
       The function must be called when the OpenGL context is changed, and
       then the initializeGL function must be called again for the new OpenGL
       context. Note that the corresponding OpenGL context must be made current
       when the function is called.
    */
    virtual void clearGL();

    virtual void setDefaultFramebufferObject(unsigned int id);

    virtual const std::string& glVendor() const = 0;

    virtual void setViewport(int x, int y, int width, int height) = 0;
    
    // Call this function instead of setViewport when the viewport is specified by the system.
    virtual void updateViewportInformation(int x, int y, int width, int height);
    
    Array4i viewport() const;
    void getViewport(int& out_x, int& out_y, int& out_width, int& out_height) const;
    double aspectRatio() const; // width / height;

    void getPerspectiveProjectionMatrix(
        double fovy, double aspect, double zNear, double zFar, Matrix4& out_matrix);
    void getOrthographicProjectionMatrix(
        double left,  double right,  double bottom,  double top,  double nearVal,  double farVal, Matrix4& out_matrix);
    
    void getViewFrustum(const SgPerspectiveCamera* camera, double& left, double& right, double& bottom, double& top) const;
    void getViewVolume(const SgOrthographicCamera* camera, float& out_left, float& out_right, float& out_bottom, float& out_top) const;
    
    bool unproject(double x, double y, double z, Vector3& out_projected) const;    

    const Vector3f& backgroundColor() const;
    void setBackgroundColor(const Vector3f& color);
    const Vector3f& defaultColor() const;
    virtual void setDefaultColor(const Vector3f& color);

    enum LightingMode {
        NormalLighting,
        MinimumLighting,
        SolidColorLighting,
        NoLighting,
        NumLightingModes
    };
    virtual void setLightingMode(LightingMode mode) = 0;
    virtual LightingMode lightingMode() const = 0;

    virtual bool isShadowCastingAvailable() const;
    virtual void setWorldLightShadowEnabled(bool on = true);
    virtual void setAdditionalLightShadowEnabled(int index, bool on = true);
    virtual void clearAdditionalLightShadows();
    virtual void setShadowAntiAliasingEnabled(bool on);
    
    virtual void setDefaultSmoothShading(bool on) = 0;
    virtual SgMaterial* defaultMaterial() = 0;
    virtual void enableTexture(bool on) = 0;
    virtual void setDefaultPointSize(double size) = 0;
    virtual void setDefaultLineWidth(double width) = 0;
    virtual void setNormalVisualizationEnabled(bool on) = 0;
    virtual void setNormalVisualizationLength(double length) = 0;
    virtual void requestToClearResources() = 0;
    virtual void enableUnusedResourceCheck(bool on) = 0;
    virtual const Vector3& pickedPoint() const = 0;
    virtual const SgNodePath& pickedNodePath() const = 0;
    virtual void setColor(const Vector3f& color) = 0;
    virtual void setUpsideDown(bool on);

    enum CullingMode {
        EnableBackFaceCulling,
        DisableBackFaceCulling,
        ForceBackfaceCulling,
        NumCullingModes,

        // deprecated
        ENABLE_BACK_FACE_CULLING = EnableBackFaceCulling,
        DISABLE_BACK_FACE_CULLING = DisableBackFaceCulling,
        FORCE_BACK_FACE_CULLING = ForceBackfaceCulling,
        N_CULLING_MODES = NumCullingModes
    };

    virtual void setBackFaceCullingMode(int mode) = 0;
    virtual int backFaceCullingMode() const = 0;

    virtual void setBoundingBoxRenderingForLightweightRenderingGroupEnabled(bool on);

    virtual void setPickingImageOutputEnabled(bool on);
    virtual bool getPickingImage(Image& out_image);

    [[deprecated("Use setNormalVisualizationEnabled and setNormaliVisualizationLength.")]]
    void showNormalVectors(double length);

private:
    class Impl;
    Impl* impl;
};

}

#endif
