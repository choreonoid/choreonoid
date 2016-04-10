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


ShaderProgram::ShaderProgram()
{

}


ShaderProgram::~ShaderProgram()
{

}


void ShaderProgram::bindGLObjects()
{

}


void ShaderProgram::setColor(const Vector4f& color)
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


void SolidColorProgram::initialize()
{
    loadVertexShader(":/Base/shader/nolighting.vert");
    loadFragmentShader(":/Base/shader/solidcolor.frag");
    link();

    NolightingProgram::initialize();

    colorLocation = getUniformLocation("color");
}


void SolidColorProgram::setColor(const Vector4f& color)
{
    glUniform4fv(colorLocation, 1, color.data());
}


void LightingProgram::setNumLights(int n)
{

}


PhongShadowProgram::PhongShadowProgram()
{
    renderingPass_ = 1;
    
    maxNumLights_ = 10;

    isShadowEnabled_ = false;
    isShadowAntiAliasingEnabled_ = false;
    shadowLightIndex_ = 0;
    shadowMapWidth_ = 1024;
    shadowMapHeight_ = 1024;
    persShadowCamera = new SgPerspectiveCamera();
    orthoShadowCamera = new SgOrthographicCamera();
    orthoShadowCamera->setHeight(5.0);
}


void PhongShadowProgram::initialize()
{
    loadVertexShader(":/Base/shader/phongshadow.vert");
    loadFragmentShader(":/Base/shader/phongshadow.frag");
    link();

    shadowMapProgram_.loadVertexShader(":/Base/shader/nolighting.vert");
    shadowMapProgram_.loadFragmentShader(":/Base/shader/shadowmap.frag");
    shadowMapProgram_.link();
    shadowMapProgram_.initialize();

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
        shadowMatrixLocation = getUniformLocation("shadowMatrix");
    }
    
    diffuseColorLocation = getUniformLocation("diffuseColor");
    ambientColorLocation = getUniformLocation("ambientColor");
    specularColorLocation = getUniformLocation("specularColor");
    emissionColorLocation = getUniformLocation("emissionColor");
    shininessLocation = getUniformLocation("shininess");
    
    numLightsLocation = getUniformLocation("numLights");
    if(numLightsLocation >= 0){
        lightHandleSets.resize(maxNumLights_);
        boost::format lightFormat("lights[%1%].");
        for(int i=0; i < maxNumLights_; ++i){
            LightHandleSet& handles = lightHandleSets[i];
            string prefix = str(lightFormat % i);
            handles.position = getUniformLocation(prefix + "position");
            handles.intensity = getUniformLocation(prefix + "intensity");
            handles.ambientIntensity = getUniformLocation(prefix + "ambientIntensity");
            handles.constantAttenuation = getUniformLocation(prefix + "constantAttenuation");
            handles.linearAttenuation = getUniformLocation(prefix + "linearAttenuation");
            handles.quadraticAttenuation = getUniformLocation(prefix + "quadraticAttenuation");
            handles.falloffAngle = getUniformLocation(prefix + "falloffAngle");
            handles.falloffExponent = getUniformLocation(prefix + "falloffExponent");
            handles.beamWidth = getUniformLocation(prefix + "beamWidth");
            handles.direction = getUniformLocation(prefix + "direction");
        }
    }

    isShadowEnabledLocation = getUniformLocation("isShadowEnabled");
    isShadowAntiAliasingEnabledLocation = getUniformLocation("isShadowAntiAliasingEnabled");
    shadowLightIndexLocation = getUniformLocation("shadowLightIndex");
    shadowMapLocation = getUniformLocation("shadowMap");
    glUniform1i(shadowMapLocation, 0);

    GLfloat border[] = { 1.0f, 0.0f, 0.0f, 0.0f };
    GLuint depthTexture;
    glGenTextures(1, &depthTexture);
    glBindTexture(GL_TEXTURE_2D, depthTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, shadowMapWidth_, shadowMapHeight_,
                 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LESS);
    
    // Assign the depth buffer texture to texture channel 0
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, depthTexture);
    
    // Create and set up the FBO
    glGenFramebuffers(1, &shadowFBO);
    glBindFramebuffer(GL_FRAMEBUFFER, shadowFBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthTexture, 0);
    
    GLenum drawBuffers[] = { GL_NONE };
    glDrawBuffers(1, drawBuffers);
    
    GLenum result = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if(result != GL_FRAMEBUFFER_COMPLETE) {
        throw Exception("Framebuffer is not complete.\n");
    }
    
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    
    shadowMapProgram_.initialize();
}


void PhongShadowProgram::use() throw (Exception)
{
    if(renderingPass_ == 0){
        shadowMapProgram_.use();
    } else {
        LightingProgram::use();
    }
}


void PhongShadowProgram::bindGLObjects()
{
    if(renderingPass_ == 0){
        glBindFramebuffer(GL_FRAMEBUFFER, shadowFBO);
        if(isShadowAntiAliasingEnabled_){
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        } else {
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        }
    } else {
        glUniform1i(isShadowEnabledLocation, isShadowEnabled_);
        if(isShadowEnabled_){
            glUniform1i(isShadowAntiAliasingEnabledLocation, isShadowAntiAliasingEnabled_);
            glUniform1i(shadowLightIndexLocation, shadowLightIndex_);
        }

        if(useUniformBlockToPassTransformationMatrices){
            transformBlockBuffer.bind(*this, 1);
            transformBlockBuffer.bindBufferBase(1);
        }
    }
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


void PhongShadowProgram::setNumLights(int n)
{
    glUniform1i(numLightsLocation, n);
}


bool PhongShadowProgram::renderLight(int index, const SgLight* light, const Affine3& T, const Affine3& viewMatrix)
{
    LightHandleSet& handles = lightHandleSets[index];

    if(const SgDirectionalLight* dirLight = dynamic_cast<const SgDirectionalLight*>(light)){
        Vector3 d = viewMatrix.linear() * T.linear() * -dirLight->direction();
        Vector4f pos(d.x(), d.y(), d.z(), 0.0f);
        glUniform4fv(handles.position, 1, pos.data());

    } else if(const SgPointLight* pointLight = dynamic_cast<const SgPointLight*>(light)){
        Vector3 p(viewMatrix * T.translation());
        Vector4f pos(p.x(), p.y(), p.z(), 1.0f);
        glUniform4fv(handles.position, 1, pos.data());
        glUniform1f(handles.constantAttenuation, pointLight->constantAttenuation());
        glUniform1f(handles.linearAttenuation, pointLight->linearAttenuation());
        glUniform1f(handles.quadraticAttenuation, pointLight->quadraticAttenuation());
        
        if(const SgSpotLight* spotLight = dynamic_cast<const SgSpotLight*>(pointLight)){
            Vector3 d = viewMatrix.linear() * T.linear() * spotLight->direction();
            Vector3f direction(d.cast<float>());
            glUniform3fv(handles.direction, 1, direction.data());
            glUniform1f(handles.falloffAngle, spotLight->cutOffAngle());
            glUniform1f(handles.falloffExponent, 4.0f);
            glUniform1f(handles.beamWidth, spotLight->beamWidth());
        }
    } else {
        return false;
    }
        
    Vector3f intensity(light->intensity() * light->color());
    glUniform3fv(handles.intensity, 1, intensity.data());
    Vector3f ambientIntensity(light->ambientIntensity() * light->color());
    glUniform3fv(handles.ambientIntensity, 1, ambientIntensity.data());

    return true;
}


void PhongShadowProgram::setTransformMatrices(const Affine3& viewMatrix, const Affine3& modelMatrix, const Matrix4& PV, const Matrix4& BPV)
{
    const Affine3f VM = (viewMatrix * modelMatrix).cast<float>();
    const Matrix3f N = VM.linear();
    const Matrix4f PVM = (PV * modelMatrix.matrix()).cast<float>();
    const Matrix4f BPVM = (BPV * modelMatrix.matrix()).cast<float>();

    if(useUniformBlockToPassTransformationMatrices){
       transformBlockBuffer.write(modelViewMatrixIndex, VM);
       transformBlockBuffer.write(normalMatrixIndex, N);
       transformBlockBuffer.write(MVPIndex, PVM);
       transformBlockBuffer.flush();
    } else {
        glUniformMatrix4fv(modelViewMatrixLocation, 1, GL_FALSE, VM.data());
        glUniformMatrix3fv(normalMatrixLocation, 1, GL_FALSE, N.data());
        glUniformMatrix4fv(MVPLocation, 1, GL_FALSE, PVM.data());
        glUniformMatrix4fv(shadowMatrixLocation, 1, GL_FALSE, BPVM.data());
    }
}


void PhongShadowProgram::setShadowEnabled(bool on)
{
    isShadowEnabled_ = on;
    if(!on){
        renderingPass_ = 1;
    }
}
