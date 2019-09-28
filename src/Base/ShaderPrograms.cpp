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

class ShaderProgramImpl
{
public:
    string vertexShader;
    string fragmentShader;
};
   

class NolightingProgramImpl
{
public:
    GLint MVPLocation;
};


class SolidColorProgramImpl
{
public:
    Vector3f color;
    GLint pointSizeLocation;
    GLint colorLocation;
    GLint colorPerVertexLocation;
    bool isColorChangable;
    bool isVertexColorEnabled;
};
    

class MinimumLightingProgramImpl
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


class BasicLightingProgramImpl
{
public:
    static const int maxNumLights = 10;

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


class MaterialLightingProgramImpl
{
public:
    static const int maxNumLights = 10;

    enum StateFlag {
        COLOR_MATERIAL,
        DIFFUSE_COLOR,
        AMBIENT_COLOR,
        EMISSION_COLOR,
        SPECULAR_COLOR,
        SHININESS,
        ALPHA,
        NUM_STATE_FLAGS
    };
    vector<bool> stateFlag;

    Vector3f diffuseColor;
    Vector3f ambientColor;
    Vector3f specularColor;
    Vector3f emissionColor;
    float shininess;
    float alpha;

    GLint diffuseColorLocation;
    GLint ambientColorLocation;
    GLint specularColorLocation;
    GLint emissionColorLocation;
    GLint shininessLocation;
    GLint alphaLocation;

    GLint isTextureEnabledLocation;
    GLint tex1Location;
    bool isTextureEnabled;

    GLint isVertexColorEnabledLocation;
    bool isVertexColorEnabled;

    void initialize(GLSLProgram& glsl);
    void setMaterial(const SgMaterial* material);
};


class PhongLightingProgramImpl
{
public:
    bool useUniformBlockToPassTransformationMatrices;
    GLSLUniformBlockBuffer transformBlockBuffer;
    GLint modelViewMatrixIndex;
    GLint normalMatrixIndex;
    GLint MVPIndex;

    GLint modelViewMatrixLocation;
    GLint normalMatrixLocation;
    GLint MVPLocation;

    void initialize(GLSLProgram& glsl);
};


class PhongShadowLightingProgramImpl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GLuint defaultFBO;

    bool isShadowAntiAliasingEnabled;
    int numShadows;
    int shadowMapWidth;
    int shadowMapHeight;
    SgPerspectiveCameraPtr persShadowCamera;  
    SgOrthographicCameraPtr orthoShadowCamera;
    ShadowMapProgram shadowMapProgram;
    
    GLint isShadowEnabledLocation;
    GLint numShadowsLocation;
    GLint isShadowAntiAliasingEnabledLocation;

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

    PhongShadowLightingProgramImpl(PhongShadowLightingProgram* self);
    void initialize(GLSLProgram& glsl);
    void initializeShadowInfo(GLSLProgram& glsl, int index);
};

}


ShaderProgram::ShaderProgram(const char* vertexShader, const char* fragmentShader)
{
    glslProgram_ = new GLSLProgram;
    impl = new ShaderProgramImpl;
    impl->vertexShader = vertexShader;
    impl->fragmentShader = fragmentShader;
}


ShaderProgram::~ShaderProgram()
{
    delete glslProgram_;
    delete impl;
}


void ShaderProgram::initialize()
{
    glslProgram_->loadVertexShader(impl->vertexShader.c_str());
    glslProgram_->loadFragmentShader(impl->fragmentShader.c_str());
    glslProgram_->link();
}


void ShaderProgram::activate()
{
    glslProgram_->use();
}


void ShaderProgram::deactivate()
{

}


void ShaderProgram::initializeFrameRendering()
{

}


void ShaderProgram::setTransform(const Matrix4& PV, const Affine3& V, const Affine3& M, const Matrix4* L)
{

}

void ShaderProgram::setMaterial(const SgMaterial* material)
{

}


void ShaderProgram::setVertexColorEnabled(bool on)
{

}


NolightingProgram::NolightingProgram()
    : NolightingProgram(":/Base/shader/nolighting.vert", ":/Base/shader/nolighting.frag")
{

}
      

NolightingProgram::NolightingProgram(const char* vertexShader, const char* fragmentShader)
    : ShaderProgram(vertexShader, fragmentShader)
{
    impl = new NolightingProgramImpl;
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


void NolightingProgram::initializeFrameRendering()
{
    glClear(GL_DEPTH_BUFFER_BIT);
}


void NolightingProgram::setTransform(const Matrix4& PV, const Affine3& V, const Affine3& M, const Matrix4* L)
{
    Matrix4f PVM;
    if(L){
        PVM = (PV * M.matrix() * (*L)).cast<float>();
    } else {
        PVM = (PV * M.matrix()).cast<float>();
    }
    glUniformMatrix4fv(impl->MVPLocation, 1, GL_FALSE, PVM.data());
}


SolidColorProgram::SolidColorProgram()
    : NolightingProgram(":/Base/shader/nolighting.vert", ":/Base/shader/solidcolor.frag")
{
    impl = new SolidColorProgramImpl;
    impl->color.setZero();
    impl->isColorChangable = true;
    impl->isVertexColorEnabled = false;
}


SolidColorProgram::~SolidColorProgram()
{
    delete impl;
}


void SolidColorProgram::initialize()
{
    NolightingProgram::initialize();

    auto& glsl = glslProgram();
    impl->pointSizeLocation = glsl.getUniformLocation("pointSize");
    impl->colorLocation = glsl.getUniformLocation("color");
    glUniform3fv(impl->colorLocation, 1, impl->color.data());
    impl->colorPerVertexLocation = glsl.getUniformLocation("colorPerVertex");
}


void SolidColorProgram::initializeFrameRendering()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}


void SolidColorProgram::activate()
{
    ShaderProgram::activate();
    
    glUniform3fv(impl->colorLocation, 1, impl->color.data());
    glUniform1i(impl->colorPerVertexLocation, impl->isVertexColorEnabled);
}


void SolidColorProgram::setMaterial(const SgMaterial* material)
{
    setColor(material->diffuseColor() + material->emissiveColor());
}


void SolidColorProgram::setColor(const Vector3f& color)
{
    if(impl->isColorChangable){
        if(color != impl->color){
            glUniform3fv(impl->colorLocation, 1, color.data());
            impl->color = color;
        }
    }
    setVertexColorEnabled(false);
}


void SolidColorProgram::setColorChangable(bool on)
{
    impl->isColorChangable = on;
}


bool SolidColorProgram::isColorChangable() const
{
    return impl->isColorChangable;
}


void SolidColorProgram::setVertexColorEnabled(bool on)
{
    if(on != impl->isVertexColorEnabled){
        glUniform1i(impl->colorPerVertexLocation, on);
        impl->isVertexColorEnabled = on;
    }
}


void SolidColorProgram::setPointSize(float s)
{
    glUniform1f(impl->pointSizeLocation, s);
}


LightingProgram::LightingProgram(const char* vertexShader, const char* fragmentShader)
    : ShaderProgram(vertexShader, fragmentShader)
{

}


void LightingProgram::initializeFrameRendering()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}


void LightingProgram::setFog(const SgFog* fog)
{

}


MinimumLightingProgram::MinimumLightingProgram()
    : LightingProgram(":/Base/shader/minlighting.vert", ":/Base/shader/minlighting.frag")
{
    impl = new MinimumLightingProgramImpl;
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


void MinimumLightingProgramImpl::initialize(GLSLProgram& glsl)
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
(const Matrix4& PV, const Affine3& V, const Affine3& M, const Matrix4* L)
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
(int index, const SgLight* light, const Affine3& T, const Affine3& view, bool shadowCasting)
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


BasicLightingProgram::BasicLightingProgram(const char* vertexShader, const char* fragmentShader)
    : LightingProgram(vertexShader, fragmentShader)
{
    impl = new BasicLightingProgramImpl;
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


void BasicLightingProgramImpl::initialize(GLSLProgram& glsl)
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
(int index, const SgLight* light, const Affine3& T, const Affine3& view, bool shadowCasting)
{
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


MaterialLightingProgram::MaterialLightingProgram(const char* vertexShader, const char* fragmentShader)
    : BasicLightingProgram(vertexShader, fragmentShader)
{
    impl = new MaterialLightingProgramImpl;
}


MaterialLightingProgram::~MaterialLightingProgram()
{
    delete impl;
}


void MaterialLightingProgram::initialize()
{
    BasicLightingProgram::initialize();
    impl->initialize(glslProgram());
}


void MaterialLightingProgramImpl::initialize(GLSLProgram& glsl)
{
    stateFlag.resize(NUM_STATE_FLAGS, false);

    diffuseColorLocation = glsl.getUniformLocation("diffuseColor");
    ambientColorLocation = glsl.getUniformLocation("ambientColor");
    specularColorLocation = glsl.getUniformLocation("specularColor");
    emissionColorLocation = glsl.getUniformLocation("emissionColor");
    shininessLocation = glsl.getUniformLocation("shininess");
    alphaLocation = glsl.getUniformLocation("alpha");

    isTextureEnabledLocation = glsl.getUniformLocation("isTextureEnabled");
    tex1Location = glsl.getUniformLocation("tex1");
    glUniform1i(tex1Location, 0);
    isTextureEnabled = false;
    glUniform1i(isTextureEnabledLocation, isTextureEnabled);

    isVertexColorEnabledLocation = glsl.getUniformLocation("isVertexColorEnabled");
    isVertexColorEnabled = false;
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


void MaterialLightingProgramImpl::setMaterial(const SgMaterial* material)
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

    float s = 127.0f * material->shininess() + 1.0f;
    if(!stateFlag[SHININESS] || shininess != s){
        glUniform1f(shininessLocation, s);
        shininess = s;
        stateFlag[SHININESS] = true;
    }

    float a = 1.0 - material->transparency();
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


PhongLightingProgram::PhongLightingProgram()
    : PhongLightingProgram(":/Base/shader/phong.vert", ":/Base/shader/phong.frag")
{

}


PhongLightingProgram::PhongLightingProgram(const char* vertexShader, const char* fragmentShader)
    : MaterialLightingProgram(vertexShader, fragmentShader)
{
    impl = new PhongLightingProgramImpl;
}
    

PhongLightingProgram::~PhongLightingProgram()
{
    delete impl;
}


void PhongLightingProgram::initialize()
{
    MaterialLightingProgram::initialize();
    impl->initialize(glslProgram());
}


void PhongLightingProgramImpl::initialize(GLSLProgram& glsl)
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
}


void PhongLightingProgram::initializeFrameRendering()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

    
void PhongLightingProgram::activate()
{
    MaterialLightingProgram::activate();
    
    if(impl->useUniformBlockToPassTransformationMatrices){
        impl->transformBlockBuffer.bind(glslProgram(), 1);
        impl->transformBlockBuffer.bindBufferBase(1);
    }
}


void PhongLightingProgram::setTransform
(const Matrix4& PV, const Affine3& V, const Affine3& M, const Matrix4* L)
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
}


PhongShadowLightingProgram::PhongShadowLightingProgram()
    : PhongLightingProgram(":/Base/shader/phongshadow.vert", ":/Base/shader/phongshadow.frag")
{
    impl = new PhongShadowLightingProgramImpl(this);
}


PhongShadowLightingProgramImpl::PhongShadowLightingProgramImpl(PhongShadowLightingProgram* self)
    : shadowMapProgram(self)
{
    defaultFBO = 0;
    numShadows = 0;
    isShadowAntiAliasingEnabled = false;
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


PhongShadowLightingProgram::~PhongShadowLightingProgram()
{
    delete impl;
}


void PhongShadowLightingProgram::setDefaultFramebufferObject(GLuint id)
{
    impl->defaultFBO = id;
}


GLuint PhongShadowLightingProgram::defaultFramebufferObject() const
{
    return impl->defaultFBO;
}


void PhongShadowLightingProgram::initialize()
{
    PhongLightingProgram::initialize();
    impl->initialize(glslProgram());
}


void PhongShadowLightingProgramImpl::initialize(GLSLProgram& glsl)
{
    numShadowsLocation = glsl.getUniformLocation("numShadows");
    shadowInfos.resize(maxNumShadows);
    for(int i=0; i < maxNumShadows; ++i){
        initializeShadowInfo(glsl, i);
    }
    glActiveTexture(GL_TEXTURE0);
    glBindFramebuffer(GL_FRAMEBUFFER, defaultFBO);

    isShadowAntiAliasingEnabledLocation = glsl.getUniformLocation("isShadowAntiAliasingEnabled");

    shadowMapProgram.initialize();
}


void PhongShadowLightingProgramImpl::initializeShadowInfo(GLSLProgram& glsl, int index)
{
    ShadowInfo& shadow = shadowInfos[index];

    shadow.shadowMatrixLocation = glsl.getUniformLocation(format("shadowMatrices[{}]", index));
    
    string prefix = format("shadows[{}].", index);
    shadow.lightIndexLocation = glsl.getUniformLocation(prefix + "lightIndex");
    shadow.shadowMapLocation = glsl.getUniformLocation(prefix + "shadowMap");

    static const GLfloat border[] = { 1.0f, 0.0f, 0.0f, 0.0f };
    glGenTextures(1, &shadow.depthTexture);
    glActiveTexture(GL_TEXTURE1 + index);
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

    glGenFramebuffers(1, &shadow.frameBuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, shadow.frameBuffer);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, shadow.depthTexture, 0);
    
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    
    GLenum result = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if(result != GL_FRAMEBUFFER_COMPLETE) {
        throw std::runtime_error(_("Framebuffer is not complete.\n"));
    }
}


void PhongShadowLightingProgram::activateShadowMapGenerationPass(int shadowIndex)
{
    if(shadowIndex >= impl->maxNumShadows){
        impl->currentShadowIndex = impl->maxNumShadows - 1;
    } else {
        impl->currentShadowIndex = shadowIndex;
    }
}


void PhongShadowLightingProgram::activateMainRenderingPass()
{
    impl->currentShadowIndex = 0;
}


void PhongShadowLightingProgram::initializeFrameRendering()
{
    glUniform1i(impl->numShadowsLocation, impl->numShadows);
    if(impl->numShadows > 0){
        glUniform1i(impl->isShadowAntiAliasingEnabledLocation, impl->isShadowAntiAliasingEnabled);
    }

    PhongLightingProgram::initializeFrameRendering();
}


void PhongShadowLightingProgram::activate()
{
    PhongLightingProgram::activate();

    glDisable(GL_CULL_FACE);
}

    
int PhongShadowLightingProgram::maxNumShadows() const
{
    return impl->maxNumShadows;
}


void PhongShadowLightingProgram::setNumShadows(int n)
{
    impl->numShadows = n;
}


ShadowMapProgram& PhongShadowLightingProgram::shadowMapProgram()
{
    return impl->shadowMapProgram;
}


void PhongShadowLightingProgram::getShadowMapSize(int& width, int& height) const
{
    width = impl->shadowMapWidth;
    height = impl->shadowMapHeight;
}


SgCamera* PhongShadowLightingProgram::getShadowMapCamera(SgLight* light, Affine3& io_T)
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


bool PhongShadowLightingProgram::setLight
(int index, const SgLight* light, const Affine3& T, const Affine3& view, bool shadowCasting)
{
    bool result = MaterialLightingProgram::setLight(index, light, T, view, shadowCasting);

    if(result && shadowCasting){
        if(impl->currentShadowIndex < impl->numShadows){
            auto& shadow = impl->shadowInfos[impl->currentShadowIndex];
            glUniform1i(shadow.shadowMapLocation, impl->currentShadowIndex + 1);
            glUniform1i(shadow.lightIndexLocation, index);
            ++impl->currentShadowIndex;
        }
    }

    return result;
}


void PhongShadowLightingProgram::setShadowMapViewProjection(const Matrix4& PV)
{
    impl->shadowInfos[impl->currentShadowIndex].BPV = impl->shadowBias * PV;
}


void PhongShadowLightingProgram::setTransform
(const Matrix4& PV, const Affine3& V, const Affine3& M, const Matrix4* L)
{
    PhongLightingProgram::setTransform(PV, V, M, L);
    
    for(int i=0; i < impl->numShadows; ++i){
        auto& shadow = impl->shadowInfos[i];
        const Matrix4f BPVM = (shadow.BPV * M.matrix()).cast<float>();
        glUniformMatrix4fv(shadow.shadowMatrixLocation, 1, GL_FALSE, BPVM.data());
    }
}


void PhongShadowLightingProgram::setShadowAntiAliasingEnabled(bool on)
{
    impl->isShadowAntiAliasingEnabled = on;
}


bool PhongShadowLightingProgram::isShadowAntiAliasingEnabled() const
{
    return impl->isShadowAntiAliasingEnabled;
}


ShadowMapProgram::ShadowMapProgram(PhongShadowLightingProgram* phongShadowProgram)
    : mainProgram(phongShadowProgram)
{

}


void ShadowMapProgram::initialize()
{
    NolightingProgram::initialize();
}


void ShadowMapProgram::initializeFrameRendering()
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
}
