/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "GLSceneRenderer.h"
#include "MessageView.h"
#include <cnoid/SceneDrawables>
#include <cnoid/SceneCameras>
#include <fmt/format.h>
#ifdef _WIN32
#include <windows.h>
#endif
#include <GL/gl.h>

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class GLSceneRendererImpl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
    GLSceneRenderer* self;
    SgGroupPtr sceneRoot;
    SgGroupPtr scene;
    Array4i viewport;
    GLfloat aspectRatio; // width / height;
    Vector3f backgroundColor;
    Vector3f defaultColor;
    GLSceneRenderer::PolygonMode polygonMode;

    GLSceneRendererImpl(GLSceneRenderer* self, SgGroup* sceneRoot);
    ~GLSceneRendererImpl();
    void onSceneGraphUpdated(const SgUpdate& update);
};

}


GLSceneRenderer::GLSceneRenderer()
{
    SgGroup* sceneRoot = new SgGroup;
    sceneRoot->setName("Root");
    impl = new GLSceneRendererImpl(this, sceneRoot);
}


GLSceneRenderer::GLSceneRenderer(SgGroup* sceneRoot)
{
    impl = new GLSceneRendererImpl(this, sceneRoot);
}


GLSceneRendererImpl::GLSceneRendererImpl(GLSceneRenderer* self, SgGroup* sceneRoot)
    : self(self),
      sceneRoot(sceneRoot)
{
    sceneRoot->sigUpdated().connect([self](const SgUpdate& update){ self->onSceneGraphUpdated(update); });

    scene = new SgGroup();
    sceneRoot->addChild(scene);

    aspectRatio = 1.0f;
    backgroundColor << 0.1f, 0.1f, 0.3f; // dark blue
    defaultColor << 1.0f, 1.0f, 1.0f;
    polygonMode = GLSceneRenderer::FILL_MODE;
}


GLSceneRenderer::~GLSceneRenderer()
{
    delete impl;
}


GLSceneRendererImpl::~GLSceneRendererImpl()
{

}


SgGroup* GLSceneRenderer::sceneRoot()
{
    return impl->sceneRoot;
}


SgGroup* GLSceneRenderer::scene()
{
    return impl->scene;
}


void GLSceneRenderer::onSceneGraphUpdated(const SgUpdate& update)
{
    if(SgImage* image = dynamic_cast<SgImage*>(update.path().front())){
        onImageUpdated(image);
    }
    SceneRenderer::onSceneGraphUpdated(update);
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


void GLSceneRenderer::setPolygonMode(PolygonMode mode)
{
    impl->polygonMode = mode;
}


GLSceneRenderer::PolygonMode GLSceneRenderer::polygonMode() const
{
    return impl->polygonMode;
}


void GLSceneRenderer::setViewport(int x, int y, int width, int height)
{
    if(height > 0){
        impl->aspectRatio = (double)width / height;
    }
    impl->viewport << x, y, width, height;
    //glViewport(x, y, width, height);
    glViewport(x, y, width, height);
}


Array4i GLSceneRenderer::viewport() const
{
    return impl->viewport;
}


void GLSceneRenderer::getViewport(int& out_x, int& out_y, int& out_width, int& out_height) const
{
    out_x = impl->viewport[0];
    out_y = impl->viewport[1];
    out_width = impl->viewport[2];
    out_height = impl->viewport[3];
}    


double GLSceneRenderer::aspectRatio() const
{
    return impl->aspectRatio;
}


bool GLSceneRenderer::initializeGL()
{
#ifndef _WIN32
    ostream& os = mvout();
    GLint major, minor;
    glGetIntegerv(GL_MAJOR_VERSION, &major);
    glGetIntegerv(GL_MINOR_VERSION, &minor);
    os << format("OpenGL version is {0}.{1}.", major, minor) << endl;
    if(major >= 2){
        os << format("GLSL version is {0}.", (const char*)glGetString(GL_SHADING_LANGUAGE_VERSION)) << endl;
    }
#endif
    return true;
}


bool GLSceneRenderer::setSwapInterval(int interval)
{
#if 0
#ifdef _WIN32
    DISPLAY_DEVICE device;
    device.cb = sizeof(DISPLAY_DEVICE);
    for (unsigned int i = 0; EnumDisplayDevices(NULL, i, &device, NULL); i++) {
        if (strstr(device.DeviceString, "VirtualBox") != NULL){
            return false;
        }
    }

    if(!wglGetProcAddress("wglGetExtensionsStringEXT"))
        return false;
    if( strstr(wglGetExtensionsStringEXT(), "WGL_EXT_swap_control" ) == 0 )
        return false;

    return wglSwapIntervalEXT(interval);
#endif
#endif
    return false;
}


int GLSceneRenderer::getSwapInterval() const
{
#if 0
#ifdef _WIN32
    return wglGetSwapIntervalEXT();
#endif
#endif
    return -1;
}


void GLSceneRenderer::clearShadows()
{

}


void GLSceneRenderer::enableShadowOfLight(int /* index */, bool /* on */)
{

}


void GLSceneRenderer::enableShadowAntiAliasing(bool /* on */)
{

}


void GLSceneRenderer::setUpsideDown(bool /* on */)
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


void GLSceneRenderer::getViewFrustum
(const SgPerspectiveCamera* camera, double& left, double& right, double& bottom, double& top) const
{
    top = camera->nearClipDistance() * tan(camera->fovy(impl->aspectRatio) / 2.0);
    bottom = -top;
    right = top * impl->aspectRatio;
    left = -right;
}


void GLSceneRenderer::getViewVolume
(const SgOrthographicCamera* camera, float& out_left, float& out_right, float& out_bottom, float& out_top) const
{
    float h = camera->height();
    out_top = h / 2.0f;
    out_bottom = -h / 2.0f;
    float w = h * impl->aspectRatio;
    out_left = -w / 2.0f;
    out_right = w / 2.0f;
}


bool GLSceneRenderer::unproject(double x, double y, double z, Vector3& out_projected) const
{
    const Array4i& vp = impl->viewport;

    Vector4 p;
    p[0] = 2.0 * (x - vp[0]) / vp[2] - 1.0;
    p[1] = 2.0 * (y - vp[1]) / vp[3] - 1.0;
    p[2] = 2.0 * z - 1.0;
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
