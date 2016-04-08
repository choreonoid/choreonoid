/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "ShaderPrograms.h"
#include <cnoid/SceneLights>
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


ShadowMapProgram::ShadowMapProgram()
{
    width_ = 1024;
    height_ = 1024;
}


void ShadowMapProgram::initialize()
{
    loadVertexShader(":/Base/shader/nolighting.vert");
    loadFragmentShader(":/Base/shader/shadowmap.frag");
    link();
        
    NolightingProgram::initialize();

    GLfloat border[] = { 1.0f, 0.0f, 0.0f, 0.0f };
    GLuint depthTexture;
    glGenTextures(1, &depthTexture);
    glBindTexture(GL_TEXTURE_2D, depthTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, width_, height_, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
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
}


void ShadowMapProgram::bindGLObjects()
{
   glBindFramebuffer(GL_FRAMEBUFFER, shadowFBO);
}


void LightingProgram::setNumLights(int n)
{

}


PhongShadowProgram::PhongShadowProgram()
{
    maxNumLights_ = 10;
    isShadowEnabled_ = false;
}


void PhongShadowProgram::initialize()
{
    loadVertexShader(":/Base/shader/phongshadow.vert");
    loadFragmentShader(":/Base/shader/phongshadow.frag");
    link();

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

    shadowMapLocation = getUniformLocation("shadowMap");
    glUniform1i(shadowMapLocation, 0);
}


void PhongShadowProgram::bindGLObjects()
{
    glUniform1i(isShadowEnabledLocation, isShadowEnabled_);
    
    if(useUniformBlockToPassTransformationMatrices){
        transformBlockBuffer.bind(*this, 1);
        transformBlockBuffer.bindBufferBase(1);
    }
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
