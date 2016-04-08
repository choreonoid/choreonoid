/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SHADER_PROGRAMS_H
#define CNOID_BASE_SHADER_PROGRAMS_H

#include "GLSLProgram.h"
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


class ShadowMapProgram : public NolightingProgram
{
    int width_;
    int height_;
    GLuint shadowFBO;
    
public:
    ShadowMapProgram();
    virtual void initialize();
    virtual void bindGLObjects();
    int width() const { return width_; }
    int height() const { return height_; }
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
public:
    bool useUniformBlockToPassTransformationMatrices;
    GLSLUniformBlockBuffer transformBlockBuffer;
    GLint modelViewMatrixIndex;
    GLint normalMatrixIndex;
    GLint MVPIndex;

    GLint modelViewMatrixLocation;
    GLint normalMatrixLocation;
    GLint MVPLocation;

    GLint shadowMatrixLocation;
    GLint shadowMapLocation;

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

    PhongShadowProgram();
    virtual void initialize();
    virtual void bindGLObjects();
    virtual void setNumLights(int n);
    virtual bool renderLight(int index, const SgLight* light, const Affine3& T, const Affine3& viewMatrix);
    virtual void setTransformMatrices(const Affine3& viewMatrix, const Affine3& modelMatrix, const Matrix4& PV, const Matrix4& BPV);
};

}

#endif
