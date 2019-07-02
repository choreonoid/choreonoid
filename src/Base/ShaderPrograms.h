/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SHADER_PROGRAMS_H
#define CNOID_BASE_SHADER_PROGRAMS_H

#include "glcore.h"
#include <cnoid/EigenTypes>
#include "exportdecl.h"

namespace cnoid {

class GLSLProgram;
class SgCamera;
class SgLight;
class SgMaterial;
class SgFog;

class ShaderProgramImpl;

class CNOID_EXPORT ShaderProgram
{
    ShaderProgram(const ShaderProgram&) = delete;

public:
    virtual ~ShaderProgram();
    GLSLProgram& glslProgram(){ return *glslProgram_; }
    virtual void initialize();

    /**
       This function is called once when the rendering starts
       if the program is used as the main shader program
    */
    virtual void initializeFrameRendering();
    
    virtual void activate();
    virtual void deactivate();

    /**
       @param PV Product of the project matrix and view matrix
       @param V The transform corresponding to the view matrix
       @param M The transform corresponding to the model matrix
       @param L The transform corresponding to the local vertex transform matrix
       @note L is used when the vertex positions are expressed as normalized ([-1.0, 1.0]) integer value
    */
    virtual void setTransform(const Matrix4& PV, const Affine3& V, const Affine3& M, const Matrix4* L = nullptr);

    virtual void setMaterial(const SgMaterial* material);
    virtual void setVertexColorEnabled(bool on);

protected:
    ShaderProgram(const char* vertexShader, const char* fragmentShader);

private:
    GLSLProgram* glslProgram_;
    ShaderProgramImpl* impl;
};


class NolightingProgramImpl;

class CNOID_EXPORT NolightingProgram : public ShaderProgram
{
    NolightingProgram(const NolightingProgram&) = delete;

public:
    NolightingProgram();
    ~NolightingProgram();
    virtual void initialize() override;
    virtual void initializeFrameRendering() override;
    virtual void setTransform(const Matrix4& PV, const Affine3& V, const Affine3& M, const Matrix4* L) override;

protected:
    NolightingProgram(const char* vertexShader, const char* fragmentShader);

private:
    NolightingProgramImpl* impl;
};


class SolidColorProgramImpl;

class CNOID_EXPORT SolidColorProgram : public NolightingProgram
{
    SolidColorProgram(const SolidColorProgram&) = delete;

public:
    SolidColorProgram();
    ~SolidColorProgram();

    virtual void initialize() override;
    virtual void initializeFrameRendering() override;
    virtual void activate() override;
    virtual void setMaterial(const SgMaterial* material) override;
    virtual void setVertexColorEnabled(bool on) override;
    
    void setColor(const Vector3f& color);
    void setColorChangable(bool on);
    bool isColorChangable() const;
    void setPointSize(float s);

private:
    SolidColorProgramImpl* impl;
};


class CNOID_EXPORT LightingProgram : public ShaderProgram
{
    LightingProgram() = default;
    LightingProgram(const LightingProgram&) = delete;

public:
    virtual void initializeFrameRendering() override;
    virtual int maxNumLights() const = 0;
    virtual bool setLight(
        int index, const SgLight* light, const Affine3& T, const Affine3& view, bool shadowCasting) = 0;
    virtual void setNumLights(int n) = 0;
    virtual void setFog(const SgFog* fog);

protected:
    LightingProgram(const char* vertexShader, const char* fragmentShader);
};


class MinimumLightingProgramImpl;

class CNOID_EXPORT MinimumLightingProgram : public LightingProgram
{
    MinimumLightingProgram(const MinimumLightingProgram&) = delete;

public:
    MinimumLightingProgram();
    ~MinimumLightingProgram();

    virtual void initialize() override;
    virtual void activate() override;
    virtual void setTransform(const Matrix4& PV, const Affine3& V, const Affine3& M, const Matrix4* L) override;
    virtual int maxNumLights() const override;
    virtual bool setLight(
        int index, const SgLight* light, const Affine3& T, const Affine3& view, bool shadowCasting) override;
    virtual void setNumLights(int n) override;
    virtual void setMaterial(const SgMaterial* material) override;

private:
    MinimumLightingProgramImpl* impl;
};


class BasicLightingProgramImpl;

class CNOID_EXPORT BasicLightingProgram : public LightingProgram
{
    BasicLightingProgram(const BasicLightingProgram&) = delete;

public:
    virtual void initialize() override;
    virtual int maxNumLights() const override;
    virtual bool setLight(
        int index, const SgLight* light, const Affine3& T, const Affine3& view, bool shadowCasting) override;
    virtual void setNumLights(int n) override;
    virtual void setFog(const SgFog* fog) override;

protected:
    BasicLightingProgram(const char* vertexShader, const char* fragmentShader);
    ~BasicLightingProgram();
    
private:
    BasicLightingProgramImpl* impl;
};


class MaterialLightingProgramImpl;

class CNOID_EXPORT MaterialLightingProgram : public BasicLightingProgram
{
    MaterialLightingProgram(const MaterialLightingProgram&) = delete;

protected:
    MaterialLightingProgram(const char* vertexShader, const char* fragmentShader);
    ~MaterialLightingProgram();
    
public:
    virtual void initialize() override;
    virtual void activate() override;
    virtual void setMaterial(const SgMaterial* material) override;
    virtual void setVertexColorEnabled(bool on) override;
    void setTextureEnabled(bool on);

private:
    MaterialLightingProgramImpl* impl;
    friend class MaterialLightingProgramImpl;
};


class PhongLightingProgramImpl;

class PhongLightingProgram : public MaterialLightingProgram
{
    PhongLightingProgram(const PhongLightingProgram&) = delete;

public:
    PhongLightingProgram();
    PhongLightingProgram(const char* vertexShader, const char* fragmentShader);
    ~PhongLightingProgram();

    virtual void initialize() override;
    virtual void initializeFrameRendering() override;
    virtual void activate() override;
    virtual void setTransform(const Matrix4& PV, const Affine3& V, const Affine3& M, const Matrix4* L) override;

private:
    PhongLightingProgramImpl* impl;
};


class PhongShadowLightingProgramImpl;
class ShadowMapProgram;

class PhongShadowLightingProgram : public PhongLightingProgram
{
    PhongShadowLightingProgram(const PhongShadowLightingProgram&) = delete;

public:
    PhongShadowLightingProgram();
    ~PhongShadowLightingProgram();

    void setDefaultFramebufferObject(GLuint id);
    GLuint defaultFramebufferObject() const;

    virtual void initialize() override;
    virtual void initializeFrameRendering() override;
    virtual void activate() override;
    virtual bool setLight(
        int index, const SgLight* light, const Affine3& T, const Affine3& view, bool shadowCasting) override;
    virtual void setTransform(const Matrix4& PV, const Affine3& V, const Affine3& M, const Matrix4* L) override;

    void activateShadowMapGenerationPass(int shadowIndex);
    void activateMainRenderingPass();

    int maxNumShadows() const;
    void setNumShadows(int n);
    ShadowMapProgram& shadowMapProgram();
    void getShadowMapSize(int& width, int& height) const;
    SgCamera* getShadowMapCamera(SgLight* light, Affine3& io_T);
    void setShadowMapViewProjection(const Matrix4& PV);
    void setShadowAntiAliasingEnabled(bool on);
    bool isShadowAntiAliasingEnabled() const;

private:
    PhongShadowLightingProgramImpl* impl;
    friend class PhongShadowLightingProgramImpl;
    friend class ShadowMapProgram;
};


class ShadowMapProgram : public NolightingProgram
{
    ShadowMapProgram(const ShadowMapProgram&) = delete;
    
public:
    ShadowMapProgram(PhongShadowLightingProgram* mainProgram);
    virtual void initialize() override;
    virtual void initializeFrameRendering() override;
    virtual void activate() override;
    virtual void deactivate() override;

private:
    PhongShadowLightingProgram* mainProgram;
};

}

#endif
