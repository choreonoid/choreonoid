/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SHADER_PROGRAMS_H
#define CNOID_BASE_SHADER_PROGRAMS_H

#include "GLSLProgram.h"
#include <cnoid/SceneCameras>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class SgLight;

class CNOID_EXPORT ShaderProgram : public GLSLProgram
{
public:
    ShaderProgram();
    virtual ~ShaderProgram();
    virtual void initialize();
    virtual void activate();
    virtual void initializeFrameRendering();
    virtual void deactivate();
    virtual void setColor(const Vector3f& color);
    virtual void enableColorArray(bool on);
};


class CNOID_EXPORT NolightingProgram : public ShaderProgram
{
    GLint MVPLocation;
    
public:
    virtual void initialize();
    void setProjectionMatrix(const Matrix4f& PVM);
};


class CNOID_EXPORT SolidColorProgram : public NolightingProgram
{
    Vector3f color_;
    GLint pointSizeLocation;
    GLint colorLocation;
    GLint colorPerVertexLocation;
    bool isColorChangable_;
    
public:
    SolidColorProgram();
    virtual void initialize() override;
    virtual void initializeFrameRendering() override;
    virtual void setPointSize(float s);
    virtual void setColor(const Vector3f& color) override;
    virtual void enableColorArray(bool on) override;
    void setColorChangable(bool on) { isColorChangable_ = on; }
    bool isColorChangable() const { return isColorChangable_; }
};


class CNOID_EXPORT LightingProgram : public ShaderProgram
{
    static const int maxNumLights_ = 10;

protected:
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
    std::vector<LightInfo> lightInfos;

    GLint maxFogDistLocation;
    GLint minFogDistLocation;
    GLint fogColorLocation;
    GLint isFogEnabledLocation;

public:
    virtual void initialize();
    int maxNumLights() const { return maxNumLights_; }
    void setNumLights(int n);
    virtual bool renderLight(int index, const SgLight* light, const Affine3& T, const Affine3& viewMatrix, bool shadowCasting);

    void setFogEnabled(bool on) {
        glUniform1i(isFogEnabledLocation, on);
    }
    void setFogColor(const Vector3f& color) {
        glUniform3fv(fogColorLocation, 1, color.data());
    }
    void setFogRange(float minDist, float maxDist){
        glUniform1f(minFogDistLocation, minDist);
        glUniform1f(maxFogDistLocation, maxDist);
    }
};


class MaterialProgram : public LightingProgram
{
protected:
    GLint diffuseColorLocation;
    GLint ambientColorLocation;
    GLint specularColorLocation;
    GLint emissionColorLocation;
    GLint shininessLocation;
    GLint alphaLocation;

    GLint isTextureEnabledLocation;
    GLint tex1Location;
    bool isTextureEnabled_;

    GLint isVertexColorEnabledLocation;
    bool isVertexColorEnabled_;

public:
    virtual void initialize() override;

    void setDiffuseColor(const Vector3f& color){
        glUniform3fv(diffuseColorLocation, 1, color.data());
    }
    void setAmbientColor(const Vector3f& color){
        glUniform3fv(ambientColorLocation, 1, color.data());
    }
    void setEmissionColor(const Vector3f& color){
        glUniform3fv(emissionColorLocation, 1, color.data());
    }
    void setSpecularColor(const Vector3f& color){
        glUniform3fv(specularColorLocation, 1, color.data());
    }
    void setShininess(float s){
        glUniform1f(shininessLocation, s);
    }
    void setAlpha(float a){
        glUniform1f(alphaLocation, a);
    }

    void setTextureEnabled(bool on){
        if(on != isTextureEnabled_){
            glUniform1i(isTextureEnabledLocation, on);
            isTextureEnabled_ = on;
        }
    }

    void setVertexColorEnabled(bool on){
        if(on != isVertexColorEnabled_){
            glUniform1i(isVertexColorEnabledLocation, on);
            isVertexColorEnabled_ = on;
        }
    }
};


class ShadowMapProgram;

class PhongShadowProgram : public MaterialProgram
{
    bool useUniformBlockToPassTransformationMatrices;
    GLSLUniformBlockBuffer transformBlockBuffer;
    GLint modelViewMatrixIndex;
    GLint normalMatrixIndex;
    GLint MVPIndex;

    GLint modelViewMatrixLocation;
    GLint normalMatrixLocation;
    GLint MVPLocation;

    bool isShadowAntiAliasingEnabled_;
    int numShadows_;
    int shadowMapWidth_;
    int shadowMapHeight_;
    SgPerspectiveCameraPtr persShadowCamera;  
    SgOrthographicCameraPtr orthoShadowCamera;
    ShadowMapProgram* shadowMapProgram_;
    
    GLint isShadowEnabledLocation;
    GLint numShadowsLocation;
    GLint isShadowAntiAliasingEnabledLocation;

    static const int maxNumShadows_ = 2;
    int currentShadowIndex;

    struct ShadowInfo {
        int lightIndex;
        GLint shadowMatrixLocation;
        GLint lightIndexLocation;
        GLint shadowMapLocation;
        GLuint depthTexture;
        GLuint frameBuffer;
        Matrix4 BPV;
    };
    std::vector<ShadowInfo> shadowInfos;

    Matrix4 shadowBias;

public:
    PhongShadowProgram();
    ~PhongShadowProgram();

    virtual void initialize() override;
    virtual void activate() override;
    virtual void initializeFrameRendering() override;
    virtual bool renderLight(int index, const SgLight* light, const Affine3& T, const Affine3& viewMatrix, bool shadowCasting) override;

    void setTransformMatrices(const Affine3& viewMatrix, const Affine3& modelMatrix, const Matrix4& PV);

    void activateShadowMapGenerationPass(int shadowIndex);
    void activateMainRenderingPass();

    int maxNumShadows() const { return maxNumShadows_; }
    void setNumShadows(int n);
    void setShadowAntiAliasingEnabled(bool on) { isShadowAntiAliasingEnabled_ = on; }
    bool isShadowAntiAliasingEnabled() const { return isShadowAntiAliasingEnabled_; }
    int shadowMapWidth() const { return shadowMapWidth_; }
    int shadowMapHeight() const { return shadowMapHeight_; }
    SgCamera* getShadowMapCamera(SgLight* light, Affine3& io_T);
    void setShadowMapViewProjection(const Matrix4& PV);
    ShadowMapProgram& shadowMapProgram() { return *shadowMapProgram_; }

private:
    void initializeShadowInfo(int index);

    friend class ShadowMapProgram;
};


class ShadowMapProgram : public NolightingProgram
{
    PhongShadowProgram* mainProgram;
    
public:
    ShadowMapProgram(PhongShadowProgram* mainProgram);
    virtual void initialize() override;
    virtual void activate() override;
    virtual void initializeFrameRendering() override;
    virtual void deactivate() override;
};

}

#endif
