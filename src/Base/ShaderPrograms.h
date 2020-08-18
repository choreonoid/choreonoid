/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SHADER_PROGRAMS_H
#define CNOID_BASE_SHADER_PROGRAMS_H

#include "glcore.h"
#include <cnoid/EigenTypes>
#include <initializer_list>
#include "exportdecl.h"

namespace cnoid {

class GLSLProgram;
class SgCamera;
class SgLight;
class SgMaterial;
class SgFog;

class CNOID_EXPORT ShaderProgram
{
    ShaderProgram(const ShaderProgram&) = delete;

public:
    virtual ~ShaderProgram();
    GLSLProgram& glslProgram(){ return *glslProgram_; }
    virtual void initialize();
    virtual void release();
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

    enum Capability {
        NoCapability = 0,
        Lighting = 1,
        Transparency = 2
    };

    int capabilities() const { return capabilities_; }
    bool hasCapability(int capability) const { return capabilities_ & capability; }

protected:
    struct ShaderSource {
        const char* filename;
        int shaderType;
    };
    
    ShaderProgram(std::initializer_list<ShaderSource> sources);
    void setCapability(int capability) { capabilities_ |= capability; }

private:
    GLSLProgram* glslProgram_;
    int capabilities_;

    class Impl;
    Impl* impl;
};


class CNOID_EXPORT NolightingProgram : public ShaderProgram
{
    NolightingProgram(const NolightingProgram&) = delete;

public:
    NolightingProgram();
    ~NolightingProgram();
    virtual void initialize() override;
    virtual void setTransform(const Matrix4& PV, const Affine3& V, const Affine3& M, const Matrix4* L) override;

protected:
    NolightingProgram(std::initializer_list<ShaderSource> sources);

private:
    class Impl;
    Impl* impl;
};


class CNOID_EXPORT SolidColorProgram : public NolightingProgram
{
    SolidColorProgram(const SolidColorProgram&) = delete;

public:
    SolidColorProgram();
    ~SolidColorProgram();

    virtual void initialize() override;
    virtual void activate() override;
    virtual void setMaterial(const SgMaterial* material) override;
    virtual void setVertexColorEnabled(bool on) override;
    
    void setColor(const Vector3f& color);
    void setColorChangable(bool on);
    bool isColorChangable() const;
    void setPointSize(float s);

protected:
    SolidColorProgram(std::initializer_list<ShaderSource> sources);

private:
    class Impl;
    Impl* impl;
};


class CNOID_EXPORT SolidPointProgram : public SolidColorProgram
{
    SolidPointProgram(const SolidPointProgram&) = delete;

public:
    SolidPointProgram();
    ~SolidPointProgram();

    virtual void initialize() override;
    virtual void activate() override;
    virtual void deactivate() override;
    virtual void setTransform(const Matrix4& PV, const Affine3& V, const Affine3& M, const Matrix4* L) override;

    void setProjectionMatrix(const Matrix4& P);    
    void setViewportSize(int width, int height);

private:
    class Impl;
    Impl* impl;
};


class CNOID_EXPORT LightingProgram : public ShaderProgram
{
    LightingProgram() = default;
    LightingProgram(const LightingProgram&) = delete;

public:
    virtual int maxNumLights() const = 0;
    virtual bool setLight(
        int index, const SgLight* light, const Affine3& T, const Affine3& view, bool shadowCasting) = 0;
    virtual void setNumLights(int n) = 0;
    virtual void setFog(const SgFog* fog);

protected:
    LightingProgram(std::initializer_list<ShaderSource> sources);
};


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
    class Impl;
    Impl* impl;
};


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
    BasicLightingProgram(std::initializer_list<ShaderSource> sources);
    ~BasicLightingProgram();
    
private:
    class Impl;
    Impl* impl;
};


class MaterialLightingProgram : public BasicLightingProgram
{
    MaterialLightingProgram(const MaterialLightingProgram&) = delete;

protected:
    MaterialLightingProgram(std::initializer_list<ShaderSource> sources);
    ~MaterialLightingProgram();
    
public:
    virtual void initialize() override;
    virtual void activate() override;
    virtual void setMaterial(const SgMaterial* material) override;
    virtual void setVertexColorEnabled(bool on) override;
    void setColorTextureIndex(int textureIndex);
    int colorTextureIndex() const;
    void setTextureEnabled(bool on);
    void setMinimumTransparency(float t);

private:
    class Impl;
    Impl* impl;
};


class ShadowMapProgram;

class FullLightingProgram : public MaterialLightingProgram
{
    FullLightingProgram(const FullLightingProgram&) = delete;

public:
    FullLightingProgram();
    FullLightingProgram(std::initializer_list<ShaderSource> sources);
    ~FullLightingProgram();

    void setDefaultFramebufferObject(GLuint id);
    GLuint defaultFramebufferObject() const;
    void setViewportSize(int width, int height);

    virtual void initialize() override;
    virtual void release() override;
    virtual void activate() override;
    virtual bool setLight(
        int index, const SgLight* light, const Affine3& T, const Affine3& view, bool shadowCasting) override;
    virtual void setTransform(const Matrix4& PV, const Affine3& V, const Affine3& M, const Matrix4* L) override;

    void setWireframeEnabled(bool on);
    
    void activateShadowMapGenerationPass(int shadowIndex);
    void activateMainRenderingPass();

    void setShadowMapTextureTopIndex(int textureIndex);
    int maxNumShadows() const;
    void setNumShadows(int n);
    ShadowMapProgram& shadowMapProgram();
    void getShadowMapSize(int& width, int& height) const;
    SgCamera* getShadowMapCamera(SgLight* light, Affine3& io_T);
    void setShadowMapViewProjection(const Matrix4& PV);
    void setShadowAntiAliasingEnabled(bool on);
    bool isShadowAntiAliasingEnabled() const;

private:
    class Impl;
    Impl* impl;
    friend class ShadowMapProgram;
};


class ShadowMapProgram : public NolightingProgram
{
    ShadowMapProgram(const ShadowMapProgram&) = delete;
    
public:
    ShadowMapProgram(FullLightingProgram* mainProgram);
    virtual void initialize() override;
    virtual void activate() override;
    void initializeShadowMapBuffer();
    virtual void deactivate() override;

private:
    FullLightingProgram* mainProgram;
};

}

#endif
