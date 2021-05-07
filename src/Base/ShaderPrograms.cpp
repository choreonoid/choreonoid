/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "ShaderPrograms.h"
#include "GLSLProgram.h"
#include <cnoid/SceneDrawables>
#include <cnoid/SceneCameras>
#include <cnoid/SceneLights>
#include <cnoid/SceneEffects>
#include <cnoid/EigenUtil>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class ShaderProgram::Impl
{
public:
    vector<ShaderSource> shaderSources;
    bool isActive;
    Impl(std::initializer_list<ShaderSource> sources);
};
   

class NolightingProgram::Impl
{
public:
    GLint MVPLocation;
};


class SolidColorProgram::Impl
{
public:
    Vector3f color;
    GLint colorLocation;
    GLint pointSizeLocation;
    bool isColorChangable;

    Impl();    
};


class SolidColorExProgram::Impl
{
public:
    GLint colorPerVertexLocation;
    GLint alphaLocation;
    float alpha;
    bool isVertexColorEnabled;
    bool alphaFlag;

    Impl();    
};


class SolidPointProgram::Impl
{
public:
    GLint projectionMatrixLocation;
    GLint modelViewMatrixLocation;
    GLint depthTextureLocation;
    GLint viewportSizeLocation;
    int viewportWidth;
    int viewportHeight;
    bool isViewportSizeInvalidated;
};


class ThickLineProgram::Impl
{
public:
    GLint lineWidthLocation;
    int lineWidth;
    GLint viewportSizeLocation;
    int viewportWidth;
    int viewportHeight;
    bool isViewportSizeInvalidated;
};


class MinimumLightingProgram::Impl
{
public:
    GLint MVPLocation;
    GLint normalMatrixLocation;

    static const int maxNumLights = 2;
    GLint numLightsLocation;
    
    struct LightInfo {
        GLint directionLocation;
        GLint intensityLocation;
        GLint ambientIntensityLocation;
    };
    vector<LightInfo> lightInfos;
    
    GLint diffuseColorLocation;
    GLint ambientColorLocation;
    Vector3f diffuseColor;
    Vector3f ambientColor;
    bool isColorApplied;

    void initialize(GLSLProgram& glsl);    
};


class BasicLightingProgram::Impl
{
public:
    static const int maxNumLights = 20;

    GLint numLightsLocation;
    
    struct LightInfo {
        GLint positionLocation;
        GLint intensityLocation;
        GLint ambientIntensityLocation;
        GLint constantAttenuationLocation;
        GLint linearAttenuationLocation;
        GLint quadraticAttenuationLocation;
        GLint cutoffAngleLocation;
        GLint beamWidthLocation;
        GLint cutoffExponentLocation;
        GLint directionLocation;
    };
    vector<LightInfo> lightInfos;
    
    GLint maxFogDistLocation;
    GLint minFogDistLocation;
    GLint fogColorLocation;
    GLint isFogEnabledLocation;

    void initialize(GLSLProgram& glsl);    
};


class MaterialLightingProgram::Impl
{
public:
    static const int maxNumLights = 10;

    enum StateFlag {
        COLOR_MATERIAL,
        DIFFUSE_COLOR,
        AMBIENT_COLOR,
        EMISSION_COLOR,
        SPECULAR_COLOR,
        SPECULAR_EXPONENT,
        ALPHA,
        NUM_STATE_FLAGS
    };
    vector<bool> stateFlag;

    Vector3f diffuseColor;
    Vector3f ambientColor;
    Vector3f specularColor;
    Vector3f emissionColor;
    float specularExponent;
    float alpha;
    float minTransparency;

    GLint diffuseColorLocation;
    GLint ambientColorLocation;
    GLint specularColorLocation;
    GLint emissionColorLocation;
    GLint specularExponentLocation;
    GLint alphaLocation;

    int colorTextureIndex;
    GLint isTextureEnabledLocation;
    GLint colorTextureLocation;
    bool isTextureEnabled;

    GLint isVertexColorEnabledLocation;
    bool isVertexColorEnabled;

    void initialize(GLSLProgram& glsl);
    void setMaterial(const SgMaterial* material);
};


class FullLightingProgram::Impl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FullLightingProgram* self;
    GLuint defaultFBO;

    bool useUniformBlockToPassTransformationMatrices;
    GLSLUniformBlockBuffer transformBlockBuffer;
    GLint modelViewMatrixIndex;
    GLint normalMatrixIndex;
    GLint MVPIndex;

    GLint modelViewMatrixLocation;
    GLint normalMatrixLocation;
    GLint MVPLocation;

    // For the wireframe overlay rendering
    int viewportWidth, viewportHeight;
    GLint viewportMatrixLocation;
    GLint isWireframeEnabledLocation;
    GLint wireframeColorLocation;
    GLint wireframeWidthLocation;
    bool isViewportMatrixInvalidated;
    bool isWireframeEnabled;
    Vector4f wireframeColor;
    float wireframeWidth;
    
    // For the shadow casting
    bool isShadowAntiAliasingEnabled;
    int shadowMapTextureTopIndex;
    int numShadows;
    int shadowMapWidth;
    int shadowMapHeight;
    SgPerspectiveCameraPtr persShadowCamera;  
    SgOrthographicCameraPtr orthoShadowCamera;
    ShadowMapProgram shadowMapProgram;
    
    GLint isShadowEnabledLocation;
    GLint numShadowsLocation;
    GLint isShadowAntiAliasingEnabledLocation;

    // This value must be same as that of shader/phongshadow.[vert/flag]
    static const int maxNumShadows = 2;
    
    int currentShadowIndex;

    struct ShadowInfo {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        int lightIndex;
        GLint shadowMatrixLocation;
        GLint lightIndexLocation;
        GLint shadowMapLocation;
        GLuint depthTexture;
        GLuint frameBuffer;
        Matrix4 BPV;
    };
    std::vector<ShadowInfo, Eigen::aligned_allocator<ShadowInfo>> shadowInfos;

    Matrix4 shadowBias;

    Impl(FullLightingProgram* self);
    void initialize(GLSLProgram& glsl);
    void initializeShadowInfo(GLSLProgram& glsl, int index);
    void activate(GLSLProgram& glsl);
    void updateShaderWireframeState();    
};

}


ShaderProgram::ShaderProgram(std::initializer_list<ShaderSource> sources)
{
    glslProgram_ = new GLSLProgram;
    capabilities_ = NoCapability;
    impl = new Impl(sources);
}


ShaderProgram::Impl::Impl(std::initializer_list<ShaderSource> sources)
    : shaderSources(sources)
{
    isActive = false;
}


ShaderProgram::~ShaderProgram()
{
    delete glslProgram_;
    delete impl;
}


void ShaderProgram::initialize()
{
    for(auto& source : impl->shaderSources){
        glslProgram_->loadShader(source.filename, source.shaderType);
    }
    glslProgram_->link();
    impl->isActive = false;
}


void ShaderProgram::release()
{
    glslProgram_->release();
    impl->isActive = false;
}


void ShaderProgram::activate()
{
    glslProgram_->use();
    impl->isActive = true;
}


void ShaderProgram::deactivate()
{
    impl->isActive = false;
}


bool ShaderProgram::isActive() const
{
    return impl->isActive;
}


void ShaderProgram::setTransform(const Matrix4& PV, const Isometry3& V, const Affine3& M, const Matrix4* L)
{

}

void ShaderProgram::setMaterial(const SgMaterial* material)
{

}


void ShaderProgram::setVertexColorEnabled(bool on)
{

}


NolightingProgram::NolightingProgram()
    : NolightingProgram(
        { { ":/Base/shader/NoLighting.vert", GL_VERTEX_SHADER },
          { ":/Base/shader/NoLighting.frag", GL_FRAGMENT_SHADER } })
{

}
      

NolightingProgram::NolightingProgram(std::initializer_list<ShaderSource> sources)
    : ShaderProgram(sources)
{
    impl = new Impl;
}


NolightingProgram::~NolightingProgram()
{
    delete impl;
}


void NolightingProgram::initialize()
{
    ShaderProgram::initialize();
    
    impl->MVPLocation = glslProgram().getUniformLocation("MVP");
}


void NolightingProgram::setTransform(const Matrix4& PV, const Isometry3& V, const Affine3& M, const Matrix4* L)
{
    Matrix4f PVM;
    if(L){
        PVM.noalias() = (PV * M.matrix() * (*L)).cast<float>();
    } else {
        PVM.noalias() = (PV * M.matrix()).cast<float>();
    }
    glUniformMatrix4fv(impl->MVPLocation, 1, GL_FALSE, PVM.data());
}


SolidColorProgram::SolidColorProgram()
    : SolidColorProgram(
        { { ":/Base/shader/SolidColor.vert", GL_VERTEX_SHADER },
          { ":/Base/shader/SolidColor.frag", GL_FRAGMENT_SHADER } })
{

}


SolidColorProgram::SolidColorProgram(std::initializer_list<ShaderSource> sources)
    : NolightingProgram(sources)
{
    impl = new Impl;
}


SolidColorProgram::Impl::Impl()
{
    color.setZero();
    isColorChangable = true;
}


SolidColorProgram::~SolidColorProgram()
{
    delete impl;
}


void SolidColorProgram::initialize()
{
    NolightingProgram::initialize();

    auto& glsl = glslProgram();
    impl->colorLocation = glsl.getUniformLocation("color");
    impl->pointSizeLocation = glsl.getUniformLocation("pointSize");
}


void SolidColorProgram::activate()
{
    ShaderProgram::activate();
    
    glUniform3fv(impl->colorLocation, 1, impl->color.data());
}


void SolidColorProgram::setMaterial(const SgMaterial* material)
{
    SolidColorProgram::setColor(material->diffuseColor() + material->emissiveColor());
}


void SolidColorProgram::setPointSize(float s)
{
    glUniform1f(impl->pointSizeLocation, s);
}


void SolidColorProgram::setColor(const Vector3f& color)
{
    if(impl->isColorChangable){
        if(color != impl->color){
            glUniform3fv(impl->colorLocation, 1, color.data());
            impl->color = color;
        }
    }
}


void SolidColorProgram::resetColor(const Vector3f& color)
{
    if(color != impl->color){
        glUniform3fv(impl->colorLocation, 1, color.data());
        impl->color = color;
    }
}


void SolidColorProgram::setColorChangable(bool on)
{
    impl->isColorChangable = on;
}


bool SolidColorProgram::isColorChangable() const
{
    return impl->isColorChangable;
}


SolidColorExProgram::SolidColorExProgram()
    : SolidColorExProgram(
        { { ":/Base/shader/SolidColor.vert", GL_VERTEX_SHADER },
          { ":/Base/shader/SolidColorEx.frag", GL_FRAGMENT_SHADER } })
{

}


SolidColorExProgram::SolidColorExProgram(std::initializer_list<ShaderSource> sources)
    : SolidColorProgram(sources)
{
    setCapability(Transparency);
    impl = new Impl;
}


SolidColorExProgram::Impl::Impl()
{
    isVertexColorEnabled = false;
}
    

SolidColorExProgram::~SolidColorExProgram()
{
    delete impl;
}


void SolidColorExProgram::initialize()
{
    SolidColorProgram::initialize();

    auto& glsl = glslProgram();
    impl->colorPerVertexLocation = glsl.getUniformLocation("colorPerVertex");
    impl->alphaLocation = glsl.getUniformLocation("alpha");
    impl->alphaFlag = false;
}


void SolidColorExProgram::activate()
{
    SolidColorProgram::activate();

    if(impl->colorPerVertexLocation >= 0){
        glUniform1i(impl->colorPerVertexLocation, impl->isVertexColorEnabled);
    }
    glUniform1f(impl->alphaLocation, impl->alpha);
    impl->alphaFlag = false;
}


void SolidColorExProgram::setMaterial(const SgMaterial* material)
{
    SolidColorExProgram::setColor(material->diffuseColor() + material->emissiveColor());

    float a = 1.0f - material->transparency();
    if(!impl->alphaFlag || impl->alpha != a){
        glUniform1f(impl->alphaLocation, a);
        impl->alpha = a;
        impl->alphaFlag = true;
    }
}


void SolidColorExProgram::setColor(const Vector3f& color)
{
    SolidColorProgram::setColor(color);
    SolidColorExProgram::setVertexColorEnabled(false);
}


void SolidColorExProgram::setVertexColorEnabled(bool on)
{
    if(on != impl->isVertexColorEnabled){
        if(impl->colorPerVertexLocation >= 0){
            glUniform1i(impl->colorPerVertexLocation, on);
        }
        impl->isVertexColorEnabled = on;
    }
}


ThickLineProgram::ThickLineProgram()
    : SolidColorExProgram(
        { { ":/Base/shader/SolidColor.vert", GL_VERTEX_SHADER },
          { ":/Base/shader/ThickLine.geom", GL_GEOMETRY_SHADER },
          { ":/Base/shader/SolidColorEx.frag", GL_FRAGMENT_SHADER } })
{
    impl = new Impl;
}


ThickLineProgram::~ThickLineProgram()
{
    delete impl;
}

    
void ThickLineProgram::initialize()
{
    SolidColorExProgram::initialize();

    impl->lineWidth = 1.0f;

    auto& glsl = glslProgram();
    impl->lineWidthLocation = glsl.getUniformLocation("lineWidth");
    impl->viewportSizeLocation = glsl.getUniformLocation("viewportSize");
    glsl.use();
}


void ThickLineProgram::activate()
{
    SolidColorExProgram::activate();

    glUniform1f(impl->lineWidthLocation, impl->lineWidth);

    if(impl->isViewportSizeInvalidated){
        glUniform2f(impl->viewportSizeLocation, impl->viewportWidth, impl->viewportHeight);
    }
}


void ThickLineProgram::setViewportSize(int width, int height)
{
    impl->viewportWidth = width;
    impl->viewportHeight = height;
    impl->isViewportSizeInvalidated = true;
}


void ThickLineProgram::setLineWidth(float width)
{
    impl->lineWidth = width;
}


SolidPointProgram::SolidPointProgram()
    : SolidColorProgram(
        { { ":/Base/shader/SolidPoint.vert", GL_VERTEX_SHADER },
          { ":/Base/shader/SolidPoint.geom", GL_GEOMETRY_SHADER },
          { ":/Base/shader/SolidPoint.frag", GL_FRAGMENT_SHADER } })
{
    impl = new Impl;
}


SolidPointProgram::~SolidPointProgram()
{
    delete impl;
}


void SolidPointProgram::initialize()
{
    SolidColorProgram::initialize();

    auto& glsl = glslProgram();
    impl->projectionMatrixLocation = glsl.getUniformLocation("projectionMatrix");
    impl->modelViewMatrixLocation = glsl.getUniformLocation("modelViewMatrix");
    impl->depthTextureLocation = glsl.getUniformLocation("depthTexture");
    impl->viewportSizeLocation = glsl.getUniformLocation("viewportSize");
    impl->isViewportSizeInvalidated = true;
    glsl.use();
    glUniform1i(impl->depthTextureLocation, 0);
}


void SolidPointProgram::activate()
{
    SolidColorProgram::activate();

    if(impl->isViewportSizeInvalidated){
        glUniform2f(impl->viewportSizeLocation, impl->viewportWidth, impl->viewportHeight);
    }
    
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
}


void SolidPointProgram::deactivate()
{
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);

    SolidColorProgram::deactivate();
}


void SolidPointProgram::setProjectionMatrix(const Matrix4& P)
{
    Matrix4f Pf(P.cast<float>());
    glUniformMatrix4fv(impl->projectionMatrixLocation, 1, GL_FALSE, Pf.data());
}


void SolidPointProgram::setTransform
(const Matrix4& PV, const Isometry3& V, const Affine3& M, const Matrix4* L)
{
    const Affine3f VM = (V * M).cast<float>();
    glUniformMatrix4fv(impl->modelViewMatrixLocation, 1, GL_FALSE, VM.data());
}


void SolidPointProgram::setViewportSize(int width, int height)
{
    impl->viewportWidth = width;
    impl->viewportHeight = height;
    impl->isViewportSizeInvalidated = true;
}


OutlineProgram::OutlineProgram()
    : SolidColorProgram(
        { { ":/Base/shader/Outline.vert", GL_VERTEX_SHADER },
          { ":/Base/shader/SolidColor.frag", GL_FRAGMENT_SHADER } })
{
    setColorChangable(false);
}


void OutlineProgram::initialize()
{
    SolidColorProgram::initialize();

    normalMatrixLocation = glslProgram().getUniformLocation("normalMatrix");
}


void OutlineProgram::setTransform
(const Matrix4& PV, const Isometry3& V, const Affine3& M, const Matrix4* L)
{
    NolightingProgram::setTransform(PV, V, M, L);

    const Matrix3f N = (V.linear() * M.linear()).cast<float>();
    glUniformMatrix3fv(normalMatrixLocation, 1, GL_FALSE, N.data());
}


void OutlineProgram::setLineWidth(float /* width */)
{

}


LightingProgram::LightingProgram(std::initializer_list<ShaderSource> sources)
    : ShaderProgram(sources)
{
    setCapability(Lighting);
}


void LightingProgram::setFog(const SgFog* fog)
{

}


MinimumLightingProgram::MinimumLightingProgram()
    : LightingProgram(
        { { ":/Base/shader/MinLighting.vert", GL_VERTEX_SHADER },
          { ":/Base/shader/MinLighting.frag", GL_FRAGMENT_SHADER } })
{
    impl = new Impl;
}


MinimumLightingProgram::~MinimumLightingProgram()
{
    delete impl;
}
    

void MinimumLightingProgram::initialize()
{
    LightingProgram::initialize();

    impl->initialize(glslProgram());
}


void MinimumLightingProgram::Impl::initialize(GLSLProgram& glsl)
{
    MVPLocation = glsl.getUniformLocation("MVP");
    normalMatrixLocation = glsl.getUniformLocation("normalMatrix");

    numLightsLocation = glsl.getUniformLocation("numLights");
    lightInfos.resize(maxNumLights);
    string lightFormat("lights[{}].");
    for(int i=0; i < maxNumLights; ++i){
        auto& light = lightInfos[i];
        string prefix = format(lightFormat, i);
        light.directionLocation = glsl.getUniformLocation(prefix + "direction");
        light.intensityLocation = glsl.getUniformLocation(prefix + "intensity");
        light.ambientIntensityLocation = glsl.getUniformLocation(prefix + "ambientIntensity");
    }

    diffuseColorLocation = glsl.getUniformLocation("diffuseColor");
    ambientColorLocation = glsl.getUniformLocation("ambientColor");
}


void MinimumLightingProgram::activate()
{
    LightingProgram::activate();

    impl->isColorApplied = false;
}
    

void MinimumLightingProgram::setTransform
(const Matrix4& PV, const Isometry3& V, const Affine3& M, const Matrix4* L)
{
    Matrix4f PVM;
    if(L){
        PVM.noalias() = (PV * M.matrix() * (*L)).cast<float>();
    } else {
        PVM.noalias() = (PV * M.matrix()).cast<float>();
    }
    glUniformMatrix4fv(impl->MVPLocation, 1, GL_FALSE, PVM.data());
    
    const Matrix3f N = (V.linear() * M.linear()).cast<float>();
    glUniformMatrix3fv(impl->normalMatrixLocation, 1, GL_FALSE, N.data());
}


int MinimumLightingProgram::maxNumLights() const
{
    return impl->maxNumLights;
}


bool MinimumLightingProgram::setLight
(int index, const SgLight* light, const Isometry3& T, const Isometry3& view, bool shadowCasting)
{
    if(index >= impl->maxNumLights){
        return false;
    }
    auto& info = impl->lightInfos[index];

    auto dirLight = dynamic_cast<const SgDirectionalLight*>(light);
    if(!dirLight){
        return false;
    }
    
    Vector3f direction = (view.linear() * T.linear() * -dirLight->direction()).cast<float>();
    glUniform3fv(info.directionLocation, 1, direction.data());
    Vector3f intensity(light->intensity() * light->color());
    glUniform3fv(info.intensityLocation, 1, intensity.data());
    Vector3f ambientIntensity(light->ambientIntensity() * light->color());
    glUniform3fv(info.ambientIntensityLocation, 1, ambientIntensity.data());

    return true;
}


void MinimumLightingProgram::setNumLights(int n)
{
    glUniform1i(impl->numLightsLocation, n);
}


void MinimumLightingProgram::setMaterial(const SgMaterial* material)
{
    const auto& dcolor = material->diffuseColor();
    if(!impl->isColorApplied || impl->diffuseColor != dcolor){
        glUniform3fv(impl->diffuseColorLocation, 1, dcolor.data());
        impl->diffuseColor = dcolor;
    }
    Vector3f acolor = material->ambientIntensity() * dcolor;
    if(!impl->isColorApplied || impl->ambientColor != acolor){
        glUniform3fv(impl->ambientColorLocation, 1, acolor.data());
        impl->ambientColor = acolor;
    }
    impl->isColorApplied = true;
}


BasicLightingProgram::BasicLightingProgram(std::initializer_list<ShaderSource> sources)
    : LightingProgram(sources)
{
    impl = new Impl;
}


BasicLightingProgram::~BasicLightingProgram()
{
    delete impl;
}

    
void BasicLightingProgram::initialize()
{
    LightingProgram::initialize();
    impl->initialize(glslProgram());
}


void BasicLightingProgram::Impl::initialize(GLSLProgram& glsl)
{    
    numLightsLocation = glsl.getUniformLocation("numLights");
    lightInfos.resize(maxNumLights);
    string lightFormat("lights[{}].");
    for(int i=0; i < maxNumLights; ++i){
        auto& light = lightInfos[i];
        string prefix = format(lightFormat, i);
        light.positionLocation = glsl.getUniformLocation(prefix + "position");
        light.intensityLocation = glsl.getUniformLocation(prefix + "intensity");
        light.ambientIntensityLocation = glsl.getUniformLocation(prefix + "ambientIntensity");
        light.constantAttenuationLocation = glsl.getUniformLocation(prefix + "constantAttenuation");
        light.linearAttenuationLocation = glsl.getUniformLocation(prefix + "linearAttenuation");
        light.quadraticAttenuationLocation = glsl.getUniformLocation(prefix + "quadraticAttenuation");
        light.cutoffAngleLocation = glsl.getUniformLocation(prefix + "cutoffAngle");
        light.beamWidthLocation = glsl.getUniformLocation(prefix + "beamWidth");
        light.cutoffExponentLocation = glsl.getUniformLocation(prefix + "cutoffExponent");
        light.directionLocation = glsl.getUniformLocation(prefix + "direction");
    }

    maxFogDistLocation = glsl.getUniformLocation("maxFogDist");
    minFogDistLocation = glsl.getUniformLocation("minFogDist");
    fogColorLocation = glsl.getUniformLocation("fogColor");
    isFogEnabledLocation = glsl.getUniformLocation("isFogEnabled");
}


void BasicLightingProgram::setNumLights(int n)
{
    glUniform1i(impl->numLightsLocation, n);
}


int BasicLightingProgram::maxNumLights() const
{
    return impl->maxNumLights;
}


bool BasicLightingProgram::setLight
(int index, const SgLight* light, const Isometry3& T, const Isometry3& view, bool shadowCasting)
{
    if(index >= impl->maxNumLights){
        return false;
    }
    
    auto& info = impl->lightInfos[index];

    if(const SgDirectionalLight* dirLight = dynamic_cast<const SgDirectionalLight*>(light)){
        Vector3 d = view.linear() * T.linear() * -dirLight->direction();
        Vector4f pos(d.x(), d.y(), d.z(), 0.0f);
        glUniform4fv(info.positionLocation, 1, pos.data());

    } else if(const SgPointLight* pointLight = dynamic_cast<const SgPointLight*>(light)){
        Vector3 p(view * T.translation());
        Vector4f pos(p.x(), p.y(), p.z(), 1.0f);
        glUniform4fv(info.positionLocation, 1, pos.data());
        glUniform1f(info.constantAttenuationLocation, pointLight->constantAttenuation());
        glUniform1f(info.linearAttenuationLocation, pointLight->linearAttenuation());
        glUniform1f(info.quadraticAttenuationLocation, pointLight->quadraticAttenuation());
        
        if(const SgSpotLight* spotLight = dynamic_cast<const SgSpotLight*>(pointLight)){
            Vector3 d = view.linear() * T.linear() * spotLight->direction();
            Vector3f direction(d.cast<float>());
            glUniform3fv(info.directionLocation, 1, direction.data());
            glUniform1f(info.cutoffAngleLocation, spotLight->cutOffAngle());
            glUniform1f(info.beamWidthLocation, spotLight->beamWidth());
            glUniform1f(info.cutoffExponentLocation, spotLight->cutOffExponent());
        }
    } else {
        return false;
    }
        
    Vector3f intensity(light->intensity() * light->color());
    glUniform3fv(info.intensityLocation, 1, intensity.data());
    Vector3f ambientIntensity(light->ambientIntensity() * light->color());
    glUniform3fv(info.ambientIntensityLocation, 1, ambientIntensity.data());

    return true;
}


void BasicLightingProgram::setFog(const SgFog* fog)
{
    if(!fog){
        glUniform1i(impl->isFogEnabledLocation, false);
    } else {
        glUniform1i(impl->isFogEnabledLocation, true);
        glUniform3fv(impl->fogColorLocation, 1, fog->color().data());
        glUniform1f(impl->minFogDistLocation, 0.0f);
        glUniform1f(impl->maxFogDistLocation, fog->visibilityRange());
    }
}


MaterialLightingProgram::MaterialLightingProgram(std::initializer_list<ShaderSource> sources)
    : BasicLightingProgram(sources)
{
    setCapability(Transparency);
    impl = new Impl;
    impl->colorTextureIndex = 1;
}


MaterialLightingProgram::~MaterialLightingProgram()
{
    delete impl;
}


void MaterialLightingProgram::setColorTextureIndex(int textureIndex)
{
    impl->colorTextureIndex = textureIndex;
}


int MaterialLightingProgram::colorTextureIndex() const
{
    return impl->colorTextureIndex;
}


void MaterialLightingProgram::initialize()
{
    BasicLightingProgram::initialize();
    impl->initialize(glslProgram());
}


void MaterialLightingProgram::Impl::initialize(GLSLProgram& glsl)
{
    stateFlag.resize(NUM_STATE_FLAGS, false);

    minTransparency = 0.0f;

    diffuseColorLocation = glsl.getUniformLocation("diffuseColor");
    ambientColorLocation = glsl.getUniformLocation("ambientColor");
    specularColorLocation = glsl.getUniformLocation("specularColor");
    emissionColorLocation = glsl.getUniformLocation("emissionColor");
    specularExponentLocation = glsl.getUniformLocation("specularExponent");
    alphaLocation = glsl.getUniformLocation("alpha");

    isTextureEnabledLocation = glsl.getUniformLocation("isTextureEnabled");
    colorTextureLocation = glsl.getUniformLocation("colorTexture");
    isTextureEnabled = false;

    isVertexColorEnabledLocation = glsl.getUniformLocation("isVertexColorEnabled");
    isVertexColorEnabled = false;


    glsl.use();
    glUniform1i(isTextureEnabledLocation, isTextureEnabled);
    glUniform1i(colorTextureLocation, colorTextureIndex);
    glUniform1i(isVertexColorEnabledLocation, isVertexColorEnabled);
}
    

void MaterialLightingProgram::activate()
{
    BasicLightingProgram::activate();
    std::fill(impl->stateFlag.begin(), impl->stateFlag.end(), false);
}


void MaterialLightingProgram::setMaterial(const SgMaterial* material)
{
    if(material){
        impl->setMaterial(material);
        setVertexColorEnabled(false);
    } else {
        std::fill(impl->stateFlag.begin(), impl->stateFlag.end(), false);
    }
}


void MaterialLightingProgram::Impl::setMaterial(const SgMaterial* material)
{
    const auto& dcolor = material->diffuseColor();
    if(!stateFlag[DIFFUSE_COLOR] || diffuseColor != dcolor){
        glUniform3fv(diffuseColorLocation, 1, dcolor.data());
        diffuseColor = dcolor;
        stateFlag[DIFFUSE_COLOR] = true;
    }

    Vector3f acolor = material->ambientIntensity() * dcolor;
    if(!stateFlag[AMBIENT_COLOR] || ambientColor != acolor){
        glUniform3fv(ambientColorLocation, 1, acolor.data());
        ambientColor = acolor;
        stateFlag[AMBIENT_COLOR] = true;
    }

    const auto& ecolor = material->emissiveColor();
    if(!stateFlag[EMISSION_COLOR] || emissionColor != ecolor){
        glUniform3fv(emissionColorLocation, 1, ecolor.data());
        emissionColor = ecolor;
        stateFlag[EMISSION_COLOR] = true;
    }

    const auto& scolor = material->specularColor();
    if(!stateFlag[SPECULAR_COLOR] || specularColor != scolor){
        glUniform3fv(specularColorLocation, 1, scolor.data());
        specularColor = scolor;
        stateFlag[SPECULAR_COLOR] = true;
    }

    float e = material->specularExponent();
    if(!stateFlag[SPECULAR_EXPONENT] || specularExponent != e){
        glUniform1f(specularExponentLocation, e);
        specularExponent = e;
        stateFlag[SPECULAR_EXPONENT] = true;
    }

    float transparency = std::max(material->transparency(), minTransparency);
    float a = 1.0 - transparency;
    if(!stateFlag[ALPHA] || alpha != a){
        glUniform1f(alphaLocation, a);
        alpha = a;
        stateFlag[ALPHA] = true;
    }
}


void MaterialLightingProgram::setVertexColorEnabled(bool on)
{
    if(on != impl->isVertexColorEnabled){
        glUniform1i(impl->isVertexColorEnabledLocation, on);
        impl->isVertexColorEnabled = on;
    }
}


void MaterialLightingProgram::setTextureEnabled(bool on)
{
    if(on != impl->isTextureEnabled){
        glUniform1i(impl->isTextureEnabledLocation, on);
        impl->isTextureEnabled = on;
    }
}


void MaterialLightingProgram::setMinimumTransparency(float t)
{
    impl->minTransparency = t;
}


FullLightingProgram::FullLightingProgram()
    : FullLightingProgram(
        { { ":/Base/shader/FullLighting.vert", GL_VERTEX_SHADER },
          { ":/Base/shader/FullLighting.geom", GL_GEOMETRY_SHADER },
          { ":/Base/shader/FullLighting.frag", GL_FRAGMENT_SHADER } })
{
    
}


FullLightingProgram::FullLightingProgram(std::initializer_list<ShaderSource> sources)
    : MaterialLightingProgram(sources)
{
    impl = new Impl(this);
}


FullLightingProgram::Impl::Impl(FullLightingProgram* self)
    : self(self),
      shadowMapProgram(self)
{
    defaultFBO = 0;

    viewportWidth = 1000;
    viewportHeight = 1000;
    isWireframeEnabled = false;
    wireframeColor << 0.4f, 0.4f, 0.4f, 0.8f;
    wireframeWidth = 0.5f;

    isShadowAntiAliasingEnabled = false;
    shadowMapTextureTopIndex = 10;
    numShadows = 0;
    shadowMapWidth = 2048;
    shadowMapHeight = 2048;
    persShadowCamera = new SgPerspectiveCamera;
    orthoShadowCamera = new SgOrthographicCamera;
    orthoShadowCamera->setHeight(15.0);
    currentShadowIndex = 0;

    shadowBias <<
        0.5, 0.0, 0.0, 0.5,
        0.0, 0.5, 0.0, 0.5,
        0.0, 0.0, 0.5, 0.5,
        0.0, 0.0, 0.0, 1.0;
}    
    

FullLightingProgram::~FullLightingProgram()
{
    delete impl;
}


void FullLightingProgram::setDefaultFramebufferObject(GLuint id)
{
    impl->defaultFBO = id;
}


GLuint FullLightingProgram::defaultFramebufferObject() const
{
    return impl->defaultFBO;
}


void FullLightingProgram::setShadowMapTextureTopIndex(int textureIndex)
{
    impl->shadowMapTextureTopIndex = textureIndex;
}


void FullLightingProgram::initialize()
{
    MaterialLightingProgram::initialize();
    impl->initialize(glslProgram());
}


void FullLightingProgram::Impl::initialize(GLSLProgram& glsl)
{
    useUniformBlockToPassTransformationMatrices = transformBlockBuffer.initialize(glsl, "TransformBlock");

    if(useUniformBlockToPassTransformationMatrices){
        modelViewMatrixIndex = transformBlockBuffer.checkUniformMatrix("modelViewMatrix");
        normalMatrixIndex = transformBlockBuffer.checkUniformMatrix("normalMatrix");
        MVPIndex = transformBlockBuffer.checkUniformMatrix("MVP");
        transformBlockBuffer.bind(glsl, 1);
        transformBlockBuffer.bindBufferBase(1);
    } else {
        modelViewMatrixLocation = glsl.getUniformLocation("modelViewMatrix");
        normalMatrixLocation = glsl.getUniformLocation("normalMatrix");
        MVPLocation = glsl.getUniformLocation("MVP");
    }

    viewportMatrixLocation = glsl.getUniformLocation("viewportMatrix");
    isViewportMatrixInvalidated = true;
    isWireframeEnabledLocation = glsl.getUniformLocation("isWireframeEnabled");
    wireframeColorLocation = glsl.getUniformLocation("wireframeColor");
    wireframeWidthLocation = glsl.getUniformLocation("wireframeWidth");

    numShadowsLocation = glsl.getUniformLocation("numShadows");
    shadowInfos.resize(maxNumShadows);
    for(int i=0; i < maxNumShadows; ++i){
        initializeShadowInfo(glsl, i);
    }
    // This is necessary to make all the shadow maps work correctly.
    // Does QOpenGLWidget do some operation on the current active texture unit
    // just after finishing the initializeGL function?
    glActiveTexture(GL_TEXTURE0);
    
    glBindFramebuffer(GL_FRAMEBUFFER, defaultFBO);

    isShadowAntiAliasingEnabledLocation = glsl.getUniformLocation("isShadowAntiAliasingEnabled");

    shadowMapProgram.initialize();

    /**
       The following code sets valid values to all the shadowMap variables
       defined in shader/phongshadow.frag. See the comment written in it.
    */
    glsl.use();
    for(int i=0; i < maxNumShadows; ++i){
        auto& shadow = shadowInfos[i];
        glUniform1i(shadow.shadowMapLocation, shadowMapTextureTopIndex + i);
    }
}


void FullLightingProgram::Impl::initializeShadowInfo(GLSLProgram& glsl, int index)
{
    ShadowInfo& shadow = shadowInfos[index];

    shadow.shadowMatrixLocation = glsl.getUniformLocation(format("shadowMatrices[{}]", index));
    
    string prefix = format("shadows[{}].", index);
    shadow.lightIndexLocation = glsl.getUniformLocation(prefix + "lightIndex");
    shadow.shadowMapLocation = glsl.getUniformLocation(prefix + "shadowMap");

    glGenFramebuffers(1, &shadow.frameBuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, shadow.frameBuffer);

    static const GLfloat border[] = { 1.0f, 0.0f, 0.0f, 0.0f };
    glGenTextures(1, &shadow.depthTexture);
    glActiveTexture(GL_TEXTURE0 + shadowMapTextureTopIndex + index);
    glBindTexture(GL_TEXTURE_2D, shadow.depthTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, shadowMapWidth, shadowMapHeight,
                 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LESS);

    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, shadow.depthTexture, 0);
    
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    
    GLenum result = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if(result != GL_FRAMEBUFFER_COMPLETE) {
        throw std::runtime_error(_("Framebuffer is not complete.\n"));
    }
}


void FullLightingProgram::release()
{
    for(int i=0; i < impl->maxNumShadows; ++i){
        auto& shadow = impl->shadowInfos[i];
        glDeleteFramebuffers(1, &shadow.frameBuffer);
        glDeleteTextures(1, &shadow.depthTexture);
    }
    impl->shadowInfos.clear();

    MaterialLightingProgram::release();
}


void FullLightingProgram::activate()
{
    MaterialLightingProgram::activate();

    impl->activate(glslProgram());
}


void FullLightingProgram::Impl::activate(GLSLProgram& glsl)
{
    if(useUniformBlockToPassTransformationMatrices){
        transformBlockBuffer.bind(glsl, 1);
        transformBlockBuffer.bindBufferBase(1);
    }

    updateShaderWireframeState();

    glDisable(GL_CULL_FACE);    
}


bool FullLightingProgram::setLight
(int index, const SgLight* light, const Isometry3& T, const Isometry3& view, bool shadowCasting)
{
    bool result = MaterialLightingProgram::setLight(index, light, T, view, shadowCasting);

    if(result && shadowCasting){
        if(impl->currentShadowIndex < impl->numShadows){
            auto& shadow = impl->shadowInfos[impl->currentShadowIndex];
            glUniform1i(shadow.lightIndexLocation, index);
            ++impl->currentShadowIndex;
        }
    }

    return result;
}


void FullLightingProgram::setTransform
(const Matrix4& PV, const Isometry3& V, const Affine3& M, const Matrix4* L)
{
    const Affine3f VM = (V * M).cast<float>();
    const Matrix3f N = VM.linear();

    Matrix4f PVM;
    if(L){
        PVM.noalias() = (PV * M.matrix() * (*L)).cast<float>();
    } else {
        PVM.noalias() = (PV * M.matrix()).cast<float>();
    }

    if(impl->useUniformBlockToPassTransformationMatrices){
        impl->transformBlockBuffer.write(impl->modelViewMatrixIndex, VM);
        impl->transformBlockBuffer.write(impl->normalMatrixIndex, N);
        impl->transformBlockBuffer.write(impl->MVPIndex, PVM);
        impl->transformBlockBuffer.flush();
    } else {
        glUniformMatrix4fv(impl->modelViewMatrixLocation, 1, GL_FALSE, VM.data());
        glUniformMatrix3fv(impl->normalMatrixLocation, 1, GL_FALSE, N.data());
        glUniformMatrix4fv(impl->MVPLocation, 1, GL_FALSE, PVM.data());
    }

    for(int i=0; i < impl->numShadows; ++i){
        auto& shadow = impl->shadowInfos[i];
        const Matrix4f BPVM = (shadow.BPV * M.matrix()).cast<float>();
        glUniformMatrix4fv(shadow.shadowMatrixLocation, 1, GL_FALSE, BPVM.data());
    }
}


void FullLightingProgram::enableWireframe(const Vector4f& color, float width)
{
    if(!impl->isWireframeEnabled || color != impl->wireframeColor || width != impl->wireframeWidth){
        impl->isWireframeEnabled = true;
        impl->wireframeColor = color;
        impl->wireframeWidth = width;
        if(isActive()){
            impl->updateShaderWireframeState();
        }
    }
}


void FullLightingProgram::disableWireframe()
{
    if(impl->isWireframeEnabled){
        impl->isWireframeEnabled = false;
        if(isActive()){
            impl->updateShaderWireframeState();
        }
    }
}


bool FullLightingProgram::isWireframeEnabled() const
{
    return impl->isWireframeEnabled;
}


void FullLightingProgram::Impl::updateShaderWireframeState()
{
    if(isWireframeEnabled && isViewportMatrixInvalidated){
        float w2 = viewportWidth / 2.0f;
        float h2 = viewportHeight / 2.0f;
        Matrix4f V;
        V <<
            w2,   0.0f, 0.0f, w2,
            0.0f, h2,   0.0f, h2,
            0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f;
        
        glUniformMatrix4fv(viewportMatrixLocation, 1, GL_FALSE, V.data());
        isViewportMatrixInvalidated = false;
    }
    glUniform1i(isWireframeEnabledLocation, isWireframeEnabled);
    if(isWireframeEnabled){
        glUniform4fv(wireframeColorLocation, 1, wireframeColor.data());
        glUniform1f(wireframeWidthLocation, wireframeWidth);
    }
}


void FullLightingProgram::setViewportSize(int width, int height)
{
    impl->viewportWidth = width;
    impl->viewportHeight = height;
    impl->isViewportMatrixInvalidated = true;
}


void FullLightingProgram::activateShadowMapGenerationPass(int shadowIndex)
{
    if(shadowIndex >= impl->maxNumShadows){
        impl->currentShadowIndex = impl->maxNumShadows - 1;
    } else {
        impl->currentShadowIndex = shadowIndex;
    }
}


void FullLightingProgram::activateMainRenderingPass()
{
    impl->currentShadowIndex = 0;

    glUniform1i(impl->numShadowsLocation, impl->numShadows);
    if(impl->numShadows > 0){
        glUniform1i(impl->isShadowAntiAliasingEnabledLocation, impl->isShadowAntiAliasingEnabled);
    }
}


int FullLightingProgram::maxNumShadows() const
{
    return impl->maxNumShadows;
}


void FullLightingProgram::setNumShadows(int n)
{
    impl->numShadows = n;
}


ShadowMapProgram* FullLightingProgram::shadowMapProgram()
{
    return &impl->shadowMapProgram;
}


void FullLightingProgram::getShadowMapSize(int& width, int& height) const
{
    width = impl->shadowMapWidth;
    height = impl->shadowMapHeight;
}


SgCamera* FullLightingProgram::getShadowMapCamera(SgLight* light, Isometry3& io_T)
{
    SgCamera* camera = 0;
    bool hasDirection = false;
    Vector3 direction;
    if(SgDirectionalLight* directional = dynamic_cast<SgDirectionalLight*>(light)){
        direction = directional->direction();
        hasDirection = true;
        camera = impl->orthoShadowCamera;
    } else if(SgSpotLight* spot = dynamic_cast<SgSpotLight*>(light)){
        direction = spot->direction();
        hasDirection = true;
        impl->persShadowCamera->setFieldOfView(spot->cutOffAngle() * 2.0);
        camera = impl->persShadowCamera;
    }
    if(hasDirection){
        Quaternion rot;
        rot.setFromTwoVectors(-Vector3::UnitZ(), direction);
        io_T.linear() = io_T.linear() * rot;
    }

    return camera;
}


void FullLightingProgram::setShadowMapViewProjection(const Matrix4& PV)
{
    impl->shadowInfos[impl->currentShadowIndex].BPV = impl->shadowBias * PV;
}


void FullLightingProgram::setShadowAntiAliasingEnabled(bool on)
{
    impl->isShadowAntiAliasingEnabled = on;
}


bool FullLightingProgram::isShadowAntiAliasingEnabled() const
{
    return impl->isShadowAntiAliasingEnabled;
}


ShadowMapProgram::ShadowMapProgram(FullLightingProgram* phongShadowProgram)
    : mainProgram(phongShadowProgram)
{

}


void ShadowMapProgram::initialize()
{
    NolightingProgram::initialize();
}


void ShadowMapProgram::initializeShadowMapBuffer()
{
    auto& mainImpl = mainProgram->impl;
    auto& shadow = mainImpl->shadowInfos[mainImpl->currentShadowIndex];
    glBindFramebuffer(GL_FRAMEBUFFER, shadow.frameBuffer);

    if(mainImpl->isShadowAntiAliasingEnabled){
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    } else {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    }

    glClear(GL_DEPTH_BUFFER_BIT);
}


void ShadowMapProgram::activate()
{
    NolightingProgram::activate();
    
    glEnable(GL_CULL_FACE);
    glCullFace(GL_FRONT);
}
    

void ShadowMapProgram::deactivate()
{
    glBindFramebuffer(GL_FRAMEBUFFER, mainProgram->impl->defaultFBO);
    glCullFace(GL_BACK);

    NolightingProgram::deactivate();
}
