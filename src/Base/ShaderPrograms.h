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
    static const int maxNumLights_ = 10;

protected:

    GLint diffuseColorLocation;
    GLint ambientColorLocation;
    GLint specularColorLocation;
    GLint emissionColorLocation;
    GLint shininessLocation;
    
public:
    virtual void initialize();
    int maxNumLights() const { return maxNumLights_; }
    virtual void setNumLights(int n) = 0;
    virtual bool renderLight(int index, const SgLight* light, const Affine3& T, const Affine3& viewMatrix, bool shadowCasting) = 0;
    virtual void setTransformMatrices(const Affine3& viewMatrix, const Affine3& modelMatrix, const Matrix4& PV) = 0;

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
    enum { SHADOWMAP_PASS, MAIN_PASS } renderingPass;

    bool useUniformBlockToPassTransformationMatrices;
    GLSLUniformBlockBuffer transformBlockBuffer;
    GLint modelViewMatrixIndex;
    GLint normalMatrixIndex;
    GLint MVPIndex;

    GLint modelViewMatrixLocation;
    GLint normalMatrixLocation;
    GLint MVPLocation;

    GLint numLightsLocation;

    struct LightInfo {
        GLint positionLocation;
        GLint intensityLocation;
        GLint ambientIntensityLocation;
        GLint constantAttenuationLocation;
        GLint linearAttenuationLocation;
        GLint quadraticAttenuationLocation;
        GLint falloffAngleLocation;
        GLint falloffExponentLocation;
        GLint beamWidthLocation;
        GLint directionLocation;
    };
    std::vector<LightInfo> lightInfos;

    bool isShadowAntiAliasingEnabled_;
    int numShadows_;
    int shadowMapWidth_;
    int shadowMapHeight_;
    SgPerspectiveCameraPtr persShadowCamera;  
    SgOrthographicCameraPtr orthoShadowCamera;
    NolightingProgram shadowMapProgram_;
    
    GLint isShadowEnabledLocation;
    GLint numShadowsLocation;
    GLint isShadowAntiAliasingEnabledLocation;

    static const int maxNumShadows_ = 1;
    int currentShadowIndex;

    struct ShadowInfo {
        int lightIndex;
        GLint shadowMatrixLocation;
        GLint lightIndexLocation;
        GLint shadowMapLocation;
        GLuint frameBuffer;
        Matrix4 BPV;
    };
    std::vector<ShadowInfo> shadowInfos;

    Matrix4 shadowBias;

public:
    PhongShadowProgram();
    virtual void initialize();
    void activateShadowMapGenerationPass(int shadowIndex);
    void activateMainRenderingPass();
    bool isShadowMapGenerationPass() const { return renderingPass == SHADOWMAP_PASS; }
    bool isMainRenderingPass() const { return renderingPass == MAIN_PASS; }
    
    virtual void use() throw (Exception);
    virtual void bindGLObjects();
    virtual void setNumLights(int n);
    virtual bool renderLight(int index, const SgLight* light, const Affine3& T, const Affine3& viewMatrix, bool shadowCasting);
    virtual void setTransformMatrices(const Affine3& viewMatrix, const Affine3& modelMatrix, const Matrix4& PV);

    int maxNumShadows() const { return maxNumShadows_; }
    void setNumShadows(int n);
    void setShadowAntiAliasingEnabled(bool on) { isShadowAntiAliasingEnabled_ = on; }
    bool isShadowAntiAliasingEnabled() const { return isShadowAntiAliasingEnabled_; }
    int shadowMapWidth() const { return shadowMapWidth_; }
    int shadowMapHeight() const { return shadowMapHeight_; }
    SgCamera* getShadowMapCamera(SgLight* light, Affine3& io_T);
    void setShadowMapViewProjection(const Matrix4& PV);
    NolightingProgram& shadowMapProgram() { return shadowMapProgram_; }

private:
    void initializeShadowInfo(int index);
};

}

#endif
