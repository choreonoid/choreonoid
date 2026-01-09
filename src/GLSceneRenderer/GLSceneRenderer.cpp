#include "GLSceneRenderer.h"
#include "GL1SceneRenderer.h"
#include "GLSLSceneRenderer.h"
#include <cnoid/SceneDrawables>
#include <cnoid/SceneCameras>
#include <cnoid/NullOut>

using namespace std;
using namespace cnoid;

namespace {

int rendererType_ = GLSceneRenderer::GLSL_RENDERER;
bool isStandardDepthBufferForced_ = false;

}

namespace cnoid {

class GLSceneRenderer::Impl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
    GLSceneRenderer* self;
    SgGroupPtr sceneRoot;
    SgGroupPtr scene;
    Vector3f backgroundColor;
    Vector3f defaultColor;
    string glslVersionString;
    ostream* os_;
    ostream& os(){ return *os_; };

    Impl(GLSceneRenderer* self, SgGroup* sceneRoot);
    ~Impl();
};

}


void GLSceneRenderer::setRendererType(int type)
{
    rendererType_ = type;
}


int GLSceneRenderer::rendererType()
{
    return rendererType_;
}


GLSceneRenderer* GLSceneRenderer::create(SgGroup* root)
{
    if(rendererType_ == GL1_RENDERER){
        return new GL1SceneRenderer(root);
    } else {
        return new GLSLSceneRenderer(root);
    }
}


GLSceneRenderer::GLSceneRenderer(SgGroup* sceneRoot)
{
    if(!sceneRoot){
        sceneRoot = new SgGroup;
        sceneRoot->setName("Root");
    }
    impl = new Impl(this, sceneRoot);
}


GLSceneRenderer::Impl::Impl(GLSceneRenderer* self, SgGroup* sceneRoot)
    : self(self),
      sceneRoot(sceneRoot)
{
    scene = new SgGroup();
    sceneRoot->insertChild(0, scene);

    int invaid = std::numeric_limits<int>::min();
    auto& vp = self->viewport_;
    vp.x = invaid;
    vp.y = invaid;
    vp.w = invaid;
    vp.h = invaid;
    self->aspectRatio_ = 1.0f;
    self->devicePixelRatio_ = 1.0f;
    backgroundColor << 0.0f, 0.0f, 0.0f; // black
    defaultColor << 1.0f, 1.0f, 1.0f;

    os_ = &nullout();
}


GLSceneRenderer::~GLSceneRenderer()
{
    delete impl;
}


GLSceneRenderer::Impl::~Impl()
{

}


void GLSceneRenderer::setOutputStream(std::ostream& os)
{
    impl->os_ = &os;
}


SgGroup* GLSceneRenderer::sceneRoot()
{
    return impl->sceneRoot;
}


SgGroup* GLSceneRenderer::scene()
{
    return impl->scene;
}


void GLSceneRenderer::clearGL()
{

}


void GLSceneRenderer::setDefaultFramebufferObject(unsigned int /* id */)
{

}


const std::string& GLSceneRenderer::glslVersionString() const
{
    return impl->glslVersionString;
}


const Vector3f& GLSceneRenderer::backgroundColor() const
{
    return impl->backgroundColor;
}


void GLSceneRenderer::setBackgroundColor(const Vector3f& color)
{
    impl->backgroundColor = color;
}


const Vector3f& GLSceneRenderer::defaultColor() const
{
    return impl->defaultColor;
}


void GLSceneRenderer::setDefaultColor(const Vector3f& color)
{
    impl->defaultColor = color;
}


void GLSceneRenderer::updateViewportInformation(int x, int y, int width, int height)
{
    if(height <= 0){
        aspectRatio_ = 1.0;
    } else {
        aspectRatio_ = static_cast<double>(width) / height;
    }
    viewport_.x = x;
    viewport_.y = y;
    viewport_.w = width;
    viewport_.h = height;
}


bool GLSceneRenderer::isShadowCastingAvailable() const
{
    return false;
}


void GLSceneRenderer::setWorldLightShadowEnabled(bool /* on */)
{

}


void GLSceneRenderer::setAdditionalLightShadowEnabled(int /* index */, bool /* on */)
{

}


void GLSceneRenderer::clearAdditionalLightShadows()
{

}


void GLSceneRenderer::setShadowAntiAliasingEnabled(bool /* on */)
{

}


void GLSceneRenderer::setUpsideDown(bool /* on */)
{

}


void GLSceneRenderer::setMsaaLevel(int /* level */)
{

}


int GLSceneRenderer::msaaLevel() const
{
    return 0;
}


void GLSceneRenderer::setDepthBufferUpdateEnabled(bool /* on */)
{

}


bool GLSceneRenderer::isDepthBufferUpdateEnabled() const
{
    return false;
}


void GLSceneRenderer::forceStandardDepthBuffer()
{
    isStandardDepthBufferForced_ = true;
}


bool GLSceneRenderer::isStandardDepthBufferForced()
{
    return isStandardDepthBufferForced_;
}


bool GLSceneRenderer::isReversedDepthBuffer() const
{
    return false;
}


void GLSceneRenderer::setInfiniteFarOverrideEnabled(bool /* on */)
{

}


bool GLSceneRenderer::isInfiniteFarOverrideEnabled() const
{
    return false;
}


void GLSceneRenderer::setBoundingBoxRenderingForLightweightRenderingGroupEnabled(bool /* on */)
{

}


void GLSceneRenderer::getPerspectiveProjectionMatrix
(double fovy, double aspect, double zNear, double zFar, Matrix4& out_matrix)
{
    const double f = 1.0 / tan(fovy / 2.0);
    out_matrix <<
        (f / aspect), 0.0, 0.0, 0.0,
        0.0, f, 0.0, 0.0,
        0.0, 0.0, ((zFar + zNear) / (zNear - zFar)), ((2.0 * zFar * zNear) / (zNear - zFar)),
        0.0, 0.0, -1.0, 0.0;
}


void GLSceneRenderer::getOrthographicProjectionMatrix
(double left,  double right,  double bottom,  double top,  double nearVal,  double farVal, Matrix4& out_matrix)
{
    const double tx = -(right + left) / (right - left);
    const double ty = -(top + bottom) / (top - bottom);
    const double tz = -(farVal + nearVal) / (farVal - nearVal);
    out_matrix <<
        (2.0 / (right - left)), 0.0 ,0.0, tx,
        0.0, (2.0 / (top - bottom)), 0.0, ty,
        0.0, 0.0, (-2.0 / (farVal - nearVal)), tz,
        0.0, 0.0, 0.0, 1.0;
}


void GLSceneRenderer::getReversedPerspectiveProjectionMatrix
(double fovy, double aspect, double zNear, double zFar, Matrix4& out_matrix)
{
    const double f = 1.0 / tan(fovy / 2.0);
    // Reversed-Z with NDC range [0, 1] (requires glClipControl GL_ZERO_TO_ONE)
    // Maps: near plane -> depth 1.0, far plane -> depth 0.0
    //
    // For reversed-Z with NDC [0,1], near->1, far->0:
    //   z_clip = A * z_eye + B, w_clip = -z_eye
    //   z_ndc = z_clip / w_clip = (A * z_eye + B) / (-z_eye)
    //   At z_eye = -n: z_ndc = (-A*n + B) / n = 1  =>  B - A*n = n
    //   At z_eye = -f: z_ndc = (-A*f + B) / f = 0  =>  B = A*f
    //   Substituting: A*f - A*n = n  =>  A = n / (f - n) = zNear / (zFar - zNear)
    //   B = A*f = zNear * zFar / (zFar - zNear)
    const double A = zNear / (zFar - zNear);
    const double B = (zFar * zNear) / (zFar - zNear);
    out_matrix <<
        (f / aspect), 0.0, 0.0, 0.0,
        0.0, f, 0.0, 0.0,
        0.0, 0.0, A, B,
        0.0, 0.0, -1.0, 0.0;
}


void GLSceneRenderer::getReversedInfinitePerspectiveProjectionMatrix
(double fovy, double aspect, double zNear, Matrix4& out_matrix)
{
    const double f = 1.0 / tan(fovy / 2.0);
    // Infinite far plane reversed-Z with NDC range [0, 1]
    // From the finite formula: A = -zNear/(zFar-zNear), B = zNear*zFar/(zFar-zNear)
    // As zFar -> infinity: A -> 0, B -> zNear
    // Maps: near plane -> depth 1.0, infinity -> depth 0.0
    out_matrix <<
        (f / aspect), 0.0, 0.0, 0.0,
        0.0, f, 0.0, 0.0,
        0.0, 0.0, 0.0, zNear,
        0.0, 0.0, -1.0, 0.0;
}


void GLSceneRenderer::getReversedOrthographicProjectionMatrix
(double left,  double right,  double bottom,  double top,  double nearVal,  double farVal, Matrix4& out_matrix)
{
    const double tx = -(right + left) / (right - left);
    const double ty = -(top + bottom) / (top - bottom);
    // Reversed-Z with NDC range [0, 1]: near maps to 1.0, far maps to 0.0
    // z_ndc = sz * z_eye + tz
    // At z_eye = -nearVal: z_ndc = 1 -> -sz*nearVal + tz = 1
    // At z_eye = -farVal:  z_ndc = 0 -> -sz*farVal + tz = 0 -> tz = sz*farVal
    // Substituting: -sz*nearVal + sz*farVal = 1 -> sz = 1/(farVal - nearVal)
    // But we need near->1, far->0, so we need to flip: sz = -1/(farVal - nearVal) = 1/(nearVal - farVal)
    // tz = sz*farVal = farVal/(nearVal - farVal)
    // Alternatively: sz = 1/(nearVal - farVal), tz = nearVal/(nearVal - farVal)
    // Check: at z=-n: -n/(n-f) + n/(n-f) = 0 ... wrong
    // Let's redo: z_ndc = sz * (-z_eye) + tz  (since eye space z is negative)
    // Actually for orthographic the standard is: z_ndc = -2/(f-n) * z_eye - (f+n)/(f-n)
    // For reversed [0,1] with near->1, far->0:
    //   At z_eye = -n: z_ndc = 1
    //   At z_eye = -f: z_ndc = 0
    // z_ndc = a * z_eye + b
    // -a*n + b = 1, -a*f + b = 0 => b = a*f
    // -a*n + a*f = 1 => a*(f-n) = 1 => a = 1/(f-n)
    // b = f/(f-n)
    // Check: z=-n => 1/(f-n)*(-n) + f/(f-n) = (-n+f)/(f-n) = 1 OK
    // Check: z=-f => 1/(f-n)*(-f) + f/(f-n) = (-f+f)/(f-n) = 0 OK
    const double sz = 1.0 / (farVal - nearVal);
    const double tz = farVal / (farVal - nearVal);
    out_matrix <<
        (2.0 / (right - left)), 0.0, 0.0, tx,
        0.0, (2.0 / (top - bottom)), 0.0, ty,
        0.0, 0.0, sz, tz,
        0.0, 0.0, 0.0, 1.0;
}


void GLSceneRenderer::getViewFrustum
(const SgPerspectiveCamera* camera, double& left, double& right, double& bottom, double& top) const
{
    top = camera->nearClipDistance() * tan(camera->fovy(aspectRatio_) / 2.0);
    bottom = -top;
    right = top * aspectRatio_;
    left = -right;
}


void GLSceneRenderer::getViewVolume
(const SgOrthographicCamera* camera, float& out_left, float& out_right, float& out_bottom, float& out_top) const
{
    float h = camera->height();
    out_top = h / 2.0f;
    out_bottom = -h / 2.0f;
    float w = h * aspectRatio_;
    out_left = -w / 2.0f;
    out_right = w / 2.0f;
}


bool GLSceneRenderer::unproject(double x, double y, double z, Vector3& out_projected) const
{
    Vector4 p;
    p[0] = 2.0 * (x - viewport_.x) / viewport_.w - 1.0;
    p[1] = 2.0 * (y - viewport_.y) / viewport_.h - 1.0;
    // For reversed depth buffer with glClipControl(GL_ZERO_TO_ONE),
    // NDC z-range is [0, 1], so no transformation is needed.
    // For standard depth buffer, NDC z-range is [-1, 1], so we transform [0,1] -> [-1,1].
    if(isReversedDepthBuffer()){
        p[2] = z;
    } else {
        p[2] = 2.0 * z - 1.0;
    }
    p[3] = 1.0;

    const Matrix4 V = currentCameraPosition().inverse().matrix();
    const Vector4 projected = (projectionMatrix() * V).inverse() * p;

    if(projected[3] == 0.0){
        return false;
    }

    out_projected.x() = projected.x() / projected[3];
    out_projected.y() = projected.y() / projected[3];
    out_projected.z() = projected.z() / projected[3];

    return true;
}


bool GLSceneRenderer::getCameraRay(double x, double y, Vector3& out_origin, Vector3& out_direction) const
{
    // Standard depth buffer: near=0.0, far=1.0
    // Use near plane and an intermediate point to calculate the ray direction
    Vector3 nearPoint, midPoint;
    if(GLSceneRenderer::unproject(x, y, 0.0, nearPoint) &&
       GLSceneRenderer::unproject(x, y, 0.5, midPoint)){
        out_origin = nearPoint;
        out_direction = (midPoint - nearPoint).normalized();
        return true;
    }
    return false;
}


void GLSceneRenderer::setPickingImageOutputEnabled(bool)
{

}
    

bool GLSceneRenderer::getPickingImage(Image&)
{
    return false;
}


void GLSceneRenderer::showNormalVectors(double length)
{
    setNormalVisualizationEnabled(length > 0.0);
    setNormalVisualizationLength(length);
}
