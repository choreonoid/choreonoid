/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SHADER_PROGRAMS_H
#define CNOID_BASE_SHADER_PROGRAMS_H

#include "GLSLProgram.h"
#include <cnoid/SceneCameras>
#include <vector>

namespace cnoid {

class SgLight;

class ShaderProgram : public GLSLProgram
{
public:
    ShaderProgram();
    virtual ~ShaderProgram();
    virtual void initialize() = 0;
    virtual void bindGLObjects();
    virtual void setColor(const Vector4f& color);
};


class NolightingProgram : public ShaderProgram
{
    GLint MVPLocation;
    
public:
    virtual void initialize();
    void setProjectionMatrix(const Matrix4f& PVM);
};


class SolidColorProgram : public NolightingProgram
{
    GLint colorLocation;
    
public:
    virtual void initialize();
    virtual void setColor(const Vector4f& color);
};


class LightingProgram : public ShaderProgram
{
protected:
    GLint diffuseColorLocation;
    GLint ambientColorLocation;
    GLint specularColorLocation;
    GLint emissionColorLocation;
    GLint shininessLocation;

    int maxNumLights_;
    
public:
    int maxNumLights() const { return maxNumLights_; }
    virtual void setNumLights(int n) = 0;
    virtual bool renderLight(int index, const SgLight* light, const Affine3& T, const Affine3& viewMatrix) = 0;
    virtual void setTransformMatrices(const Affine3& viewMatrix, const Affine3& modelMatrix, const Matrix4& PV, const Matrix4& BPV) = 0;

    void setDiffuseColor(const Vector4f& color){
        glUniform3fv(diffuseColorLocation, 1, color.data());
    }
    void setAmbientColor(const Vector4f& color){
        glUniform3fv(ambientColorLocation, 1, color.data());
    }
    void setEmissionColor(const Vector4f& color){
        glUniform3fv(emissionColorLocation, 1, color.data());
    }
    void setSpecularColor(const Vector4f& color){
        glUniform3fv(specularColorLocation, 1, color.data());
    }
    void setShininess(float s){
        glUniform1f(shininessLocation, s);
    }
};


class PhongShadowProgram : public LightingProgram
{
    int renderingPass_;

    bool useUniformBlockToPassTransformationMatrices;
    GLSLUniformBlockBuffer transformBlockBuffer;
    GLint modelViewMatrixIndex;
    GLint normalMatrixIndex;
    GLint MVPIndex;

    GLint modelViewMatrixLocation;
    GLint normalMatrixLocation;
    GLint MVPLocation;

    GLint numLightsLocation;

    struct LightHandleSet {
        GLint position;
        GLint intensity;
        GLint ambientIntensity;
        GLint constantAttenuation;
        GLint linearAttenuation;
        GLint quadraticAttenuation;
        GLint falloffAngle;
        GLint falloffExponent;
        GLint beamWidth;
        GLint direction;
    };
    std::vector<LightHandleSet> lightHandleSets;

    bool isShadowEnabled_;
    GLint isShadowEnabledLocation;
    bool isShadowAntiAliasingEnabled_;
    GLint isShadowAntiAliasingEnabledLocation;
    int shadowLightIndex_;
    GLint shadowLightIndexLocation;
    GLint shadowMatrixLocation;
    GLint shadowMapLocation;
    int shadowMapWidth_;
    int shadowMapHeight_;
    GLuint shadowFBO;
    SgPerspectiveCameraPtr persShadowCamera;  
    SgOrthographicCameraPtr orthoShadowCamera;
    NolightingProgram shadowMapProgram_;

public:
    PhongShadowProgram();
    virtual void initialize();
    void setRenderingPass(int pass) { renderingPass_ = pass; }
    int renderingPass() const { return renderingPass_; }
    virtual void use() throw (Exception);
    virtual void bindGLObjects();
    virtual void setNumLights(int n);
    virtual bool renderLight(int index, const SgLight* light, const Affine3& T, const Affine3& viewMatrix);
    virtual void setTransformMatrices(const Affine3& viewMatrix, const Affine3& modelMatrix, const Matrix4& PV, const Matrix4& BPV);

    bool isShadowEnabled() const { return isShadowEnabled_; }
    void setShadowEnabled(bool on);
    int shadowLightIndex() const { return shadowLightIndex_; }
    void setShadowLight(int index) { shadowLightIndex_ = index; }
    void setShadowAntiAliasingEnabled(bool on) { isShadowAntiAliasingEnabled_ = on; }
    bool isShadowAntiAliasingEnabled() const { return isShadowAntiAliasingEnabled_; }
    int shadowMapWidth() const { return shadowMapWidth_; }
    int shadowMapHeight() const { return shadowMapHeight_; }
    SgCamera* getShadowMapCamera(SgLight* light, Affine3& io_T);
    NolightingProgram& shadowMapProgram() { return shadowMapProgram_; }
};

}

#endif
