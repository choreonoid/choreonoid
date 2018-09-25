/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "ShaderPrograms.h"
#include <cnoid/SceneLights>
#include <cnoid/EigenUtil>
#include <boost/format.hpp>

using namespace std;
using namespace cnoid;
using boost::format;


ShaderProgram::ShaderProgram()
{

}


ShaderProgram::~ShaderProgram()
{

}


void ShaderProgram::initialize()
{

}


void ShaderProgram::activate()
{
    use();
}


void ShaderProgram::deactivate()
{

}


void ShaderProgram::initializeFrameRendering()
{

}


void ShaderProgram::setColor(const Vector3f& color)
{

}


void ShaderProgram::enableColorArray(bool on)
{

}


void NolightingProgram::initialize()
{
    MVPLocation = getUniformLocation("MVP");
}


void NolightingProgram::setProjectionMatrix(const Matrix4f& PVM)
{
    glUniformMatrix4fv(MVPLocation, 1, GL_FALSE, PVM.data());
}


SolidColorProgram::SolidColorProgram()
{
    color_.setZero();
    isColorChangable_ = true;
}


void SolidColorProgram::initialize()
{
    loadVertexShader(":/Base/shader/nolighting.vert");
    loadFragmentShader(":/Base/shader/solidcolor.frag");
    link();
    use();

    NolightingProgram::initialize();

    pointSizeLocation = getUniformLocation("pointSize");
    colorLocation = getUniformLocation("color");
    glUniform3fv(colorLocation, 1, color_.data());
    colorPerVertexLocation = getUniformLocation("colorPerVertex");
}


void SolidColorProgram::initializeFrameRendering()
{
    glUniform1i(colorPerVertexLocation, false);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}


void SolidColorProgram::setPointSize(float s)
{
    glUniform1f(pointSizeLocation, s);
}


void SolidColorProgram::setColor(const Vector3f& color)
{
    if(isColorChangable_){
        if(color != color_){
            glUniform3fv(colorLocation, 1, color.data());
            glUniform1i(colorPerVertexLocation, false);
            color_ = color;
        }
    }
}


void SolidColorProgram::enableColorArray(bool on)
{
    glUniform1i(colorPerVertexLocation, on);
}
    

void LightingProgram::initialize()
{
    numLightsLocation = getUniformLocation("numLights");
    lightInfos.resize(maxNumLights());
    format lightFormat("lights[%1%].");
    for(int i=0; i < maxNumLights(); ++i){
        LightInfo& light = lightInfos[i];
        string prefix = str(lightFormat % i);
        light.positionLocation = getUniformLocation(prefix + "position");
        light.intensityLocation = getUniformLocation(prefix + "intensity");
        light.ambientIntensityLocation = getUniformLocation(prefix + "ambientIntensity");
        light.constantAttenuationLocation = getUniformLocation(prefix + "constantAttenuation");
        light.linearAttenuationLocation = getUniformLocation(prefix + "linearAttenuation");
        light.quadraticAttenuationLocation = getUniformLocation(prefix + "quadraticAttenuation");
        light.cutoffAngleLocation = getUniformLocation(prefix + "cutoffAngle");
        light.beamWidthLocation = getUniformLocation(prefix + "beamWidth");
        light.cutoffExponentLocation = getUniformLocation(prefix + "cutoffExponent");
        light.directionLocation = getUniformLocation(prefix + "direction");
    }

    maxFogDistLocation = getUniformLocation("maxFogDist");
    minFogDistLocation = getUniformLocation("minFogDist");
    fogColorLocation = getUniformLocation("fogColor");
    isFogEnabledLocation = getUniformLocation("isFogEnabled");
}    
    


void LightingProgram::setNumLights(int n)
{
    glUniform1i(numLightsLocation, n);
}


bool LightingProgram::renderLight(int index, const SgLight* light, const Affine3& T, const Affine3& viewMatrix, bool shadowCasting)
{
    LightInfo& info = lightInfos[index];

    if(const SgDirectionalLight* dirLight = dynamic_cast<const SgDirectionalLight*>(light)){
        Vector3 d = viewMatrix.linear() * T.linear() * -dirLight->direction();
        Vector4f pos(d.x(), d.y(), d.z(), 0.0f);
        glUniform4fv(info.positionLocation, 1, pos.data());

    } else if(const SgPointLight* pointLight = dynamic_cast<const SgPointLight*>(light)){
        Vector3 p(viewMatrix * T.translation());
        Vector4f pos(p.x(), p.y(), p.z(), 1.0f);
        glUniform4fv(info.positionLocation, 1, pos.data());
        glUniform1f(info.constantAttenuationLocation, pointLight->constantAttenuation());
        glUniform1f(info.linearAttenuationLocation, pointLight->linearAttenuation());
        glUniform1f(info.quadraticAttenuationLocation, pointLight->quadraticAttenuation());
        
        if(const SgSpotLight* spotLight = dynamic_cast<const SgSpotLight*>(pointLight)){
            Vector3 d = viewMatrix.linear() * T.linear() * spotLight->direction();
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


void MaterialProgram::initialize()
{
    LightingProgram::initialize();
    
    diffuseColorLocation = getUniformLocation("diffuseColor");
    ambientColorLocation = getUniformLocation("ambientColor");
    specularColorLocation = getUniformLocation("specularColor");
    emissionColorLocation = getUniformLocation("emissionColor");
    shininessLocation = getUniformLocation("shininess");
    alphaLocation = getUniformLocation("alpha");

    isTextureEnabledLocation = getUniformLocation("isTextureEnabled");
    tex1Location = getUniformLocation("tex1");
    glUniform1i(tex1Location, 0);
    isTextureEnabled_ = false;
    glUniform1i(isTextureEnabledLocation, isTextureEnabled_);

    isVertexColorEnabledLocation = getUniformLocation("isVertexColorEnabled");
    isVertexColorEnabled_ = false;
    glUniform1i(isVertexColorEnabledLocation, isVertexColorEnabled_);
}


PhongShadowProgram::PhongShadowProgram()
{
    numShadows_ = 0;
    isShadowAntiAliasingEnabled_ = false;
    shadowMapWidth_ = 2048;
    shadowMapHeight_ = 2048;
    persShadowCamera = new SgPerspectiveCamera();
    orthoShadowCamera = new SgOrthographicCamera();
    orthoShadowCamera->setHeight(15.0);
    currentShadowIndex = 0;

    shadowBias <<
        0.5, 0.0, 0.0, 0.5,
        0.0, 0.5, 0.0, 0.5,
        0.0, 0.0, 0.5, 0.5,
        0.0, 0.0, 0.0, 1.0;

    shadowMapProgram_ = new ShadowMapProgram(this);
}


PhongShadowProgram::~PhongShadowProgram()
{
    delete shadowMapProgram_;
}


void PhongShadowProgram::initialize()
{
    loadVertexShader(":/Base/shader/phongshadow.vert");
    loadFragmentShader(":/Base/shader/phongshadow.frag");
    link();
    use();

    MaterialProgram::initialize();

    useUniformBlockToPassTransformationMatrices = transformBlockBuffer.initialize(*this, "TransformBlock");

    if(useUniformBlockToPassTransformationMatrices){
        modelViewMatrixIndex = transformBlockBuffer.checkUniformMatrix("modelViewMatrix");
        normalMatrixIndex = transformBlockBuffer.checkUniformMatrix("normalMatrix");
        MVPIndex = transformBlockBuffer.checkUniformMatrix("MVP");
        transformBlockBuffer.bind(*this, 1);
        transformBlockBuffer.bindBufferBase(1);
    } else {
        modelViewMatrixLocation = getUniformLocation("modelViewMatrix");
        normalMatrixLocation = getUniformLocation("normalMatrix");
        MVPLocation = getUniformLocation("MVP");
    }
    
    numShadowsLocation = getUniformLocation("numShadows");
    shadowInfos.resize(maxNumShadows_);
    for(int i=0; i < maxNumShadows_; ++i){
        initializeShadowInfo(i);
    }
    glActiveTexture(GL_TEXTURE0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    isShadowAntiAliasingEnabledLocation = getUniformLocation("isShadowAntiAliasingEnabled");

    shadowMapProgram_->initialize();
}


void PhongShadowProgram::initializeShadowInfo(int index)
{
    ShadowInfo& shadow = shadowInfos[index];

    format shadowMatrixFormat("shadowMatrices[%1%]");
    shadow.shadowMatrixLocation = getUniformLocation(str(format("shadowMatrices[%1%]") % index));
    
    format shadowFormat("shadows[%1%].");
    string prefix = str(format("shadows[%1%].") % index);
    shadow.lightIndexLocation = getUniformLocation(prefix + "lightIndex");
    shadow.shadowMapLocation = getUniformLocation(prefix + "shadowMap");

    static const GLfloat border[] = { 1.0f, 0.0f, 0.0f, 0.0f };
    glGenTextures(1, &shadow.depthTexture);
    glActiveTexture(GL_TEXTURE1 + index);
    glBindTexture(GL_TEXTURE_2D, shadow.depthTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, shadowMapWidth_, shadowMapHeight_,
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
        throw Exception("Framebuffer is not complete.\n");
    }
}


void PhongShadowProgram::activateShadowMapGenerationPass(int shadowIndex)
{
    if(shadowIndex >= maxNumShadows_){
        currentShadowIndex = maxNumShadows_ - 1;
    } else {
        currentShadowIndex = shadowIndex;
    }
}


void PhongShadowProgram::activateMainRenderingPass()
{
    currentShadowIndex = 0;
}


void PhongShadowProgram::activate()
{
    LightingProgram::activate();
    
    glDisable(GL_CULL_FACE);
}


void PhongShadowProgram::initializeFrameRendering()
{
    glUniform1i(numShadowsLocation, numShadows_);
    if(numShadows_ > 0){
        glUniform1i(isShadowAntiAliasingEnabledLocation, isShadowAntiAliasingEnabled_);
    }

    if(useUniformBlockToPassTransformationMatrices){
        transformBlockBuffer.bind(*this, 1);
        transformBlockBuffer.bindBufferBase(1);
    }

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

    
SgCamera* PhongShadowProgram::getShadowMapCamera(SgLight* light, Affine3& io_T)
{
    SgCamera* camera = 0;
    bool hasDirection = false;
    Vector3 direction;
    if(SgDirectionalLight* directional = dynamic_cast<SgDirectionalLight*>(light)){
        direction = directional->direction();
        hasDirection = true;
        camera = orthoShadowCamera;
    } else if(SgSpotLight* spot = dynamic_cast<SgSpotLight*>(light)){
        direction = spot->direction();
        hasDirection = true;
        persShadowCamera->setFieldOfView(spot->cutOffAngle() * 2.0);
        camera = persShadowCamera;
    }
    if(hasDirection){
        Quaternion rot;
        rot.setFromTwoVectors(-Vector3::UnitZ(), direction);
        io_T.linear() = io_T.linear() * rot;
    }

    return camera;
}


bool PhongShadowProgram::renderLight(int index, const SgLight* light, const Affine3& T, const Affine3& viewMatrix, bool shadowCasting)
{
    bool result = LightingProgram::renderLight(index, light, T, viewMatrix, shadowCasting);

    if(result && shadowCasting){
        if(currentShadowIndex < numShadows_){
            ShadowInfo& shadow = shadowInfos[currentShadowIndex];
            glUniform1i(shadow.shadowMapLocation, currentShadowIndex + 1);
            glUniform1i(shadow.lightIndexLocation, index);
            ++currentShadowIndex;
        }
    }

    return result;
}


void PhongShadowProgram::setNumShadows(int n)
{
    numShadows_ = n;
}


void PhongShadowProgram::setShadowMapViewProjection(const Matrix4& PV)
{
    shadowInfos[currentShadowIndex].BPV = shadowBias * PV;
}


void PhongShadowProgram::setTransformMatrices(const Affine3& viewMatrix, const Affine3& modelMatrix, const Matrix4& PV)
{
    const Affine3f VM = (viewMatrix * modelMatrix).cast<float>();
    const Matrix3f N = VM.linear();
    const Matrix4f PVM = (PV * modelMatrix.matrix()).cast<float>();

    if(useUniformBlockToPassTransformationMatrices){
       transformBlockBuffer.write(modelViewMatrixIndex, VM);
       transformBlockBuffer.write(normalMatrixIndex, N);
       transformBlockBuffer.write(MVPIndex, PVM);
       transformBlockBuffer.flush();
    } else {
        glUniformMatrix4fv(modelViewMatrixLocation, 1, GL_FALSE, VM.data());
        glUniformMatrix3fv(normalMatrixLocation, 1, GL_FALSE, N.data());
        glUniformMatrix4fv(MVPLocation, 1, GL_FALSE, PVM.data());
        for(int i=0; i < numShadows_; ++i){
            ShadowInfo& shadow = shadowInfos[i];
            const Matrix4f BPVM = (shadow.BPV * modelMatrix.matrix()).cast<float>();
            glUniformMatrix4fv(shadow.shadowMatrixLocation, 1, GL_FALSE, BPVM.data());
        }
    }
}


ShadowMapProgram::ShadowMapProgram(PhongShadowProgram* phongShadowProgram)
    : mainProgram(phongShadowProgram)
{

}


void ShadowMapProgram::initialize()
{
    loadVertexShader(":/Base/shader/nolighting.vert");
    loadFragmentShader(":/Base/shader/shadowmap.frag");
    link();
    use();
    
    NolightingProgram::initialize();
}


void ShadowMapProgram::activate()
{
    NolightingProgram::activate();
    
    glEnable(GL_CULL_FACE);
    glCullFace(GL_FRONT);
}
    

void ShadowMapProgram::initializeFrameRendering()
{
    PhongShadowProgram::ShadowInfo& shadow = mainProgram->shadowInfos[mainProgram->currentShadowIndex];
    glBindFramebuffer(GL_FRAMEBUFFER, shadow.frameBuffer);
        
    if(mainProgram->isShadowAntiAliasingEnabled_){
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    } else {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    }

    glClear(GL_DEPTH_BUFFER_BIT);
}


void ShadowMapProgram::deactivate()
{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glCullFace(GL_BACK);
}
