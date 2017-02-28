/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PHENOMENON_PLUGIN_PARTICLES_PROGRAM_H
#define CNOID_PHENOMENON_PLUGIN_PARTICLES_PROGRAM_H

#include "SceneParticles.h"
#include <cnoid/GLSLSceneRenderer>
#include <cnoid/ShaderPrograms>
#include <memory>

namespace cnoid {

class ParticlesProgramBase
{
public:
    ParticlesProgramBase(GLSLSceneRenderer* renderer);
    
    void requestRendering(SceneParticles* particles, std::function<void()> renderingFunction);

    void setTime(float time){
        glUniform1f(timeLocation, time);
    }
    static float random(float max = 1.0f) {
        return ((float)rand() / RAND_MAX) * max;
    }

protected:
    virtual bool initializeRendering(SceneParticles* particles) = 0;
    virtual ShaderProgram* shaderProgram() = 0;

private:
    enum State { NOT_INITIALIZED, INITIALIZED, FAILED } initializationState;
    GLSLSceneRenderer* renderer;
    GLint modelViewMatrixLocation;
    GLint projectionMatrixLocation;
    GLint pointSizeLocation;
    GLint angle2pixelsLocation;
    GLint timeLocation;
    GLint particleTexLocation;
    GLuint textureId;

    void render(SceneParticles* particles, const Matrix4f& MV, const std::function<void()>& renderingFunction);
};

    
class ParticlesProgram : public LightingProgram, public ParticlesProgramBase
{
public:
    ParticlesProgram(GLSLSceneRenderer* renderer);

protected:
    virtual ShaderProgram* shaderProgram() { return this; }
};


/*
class LuminousParticlesProgram : public ShaderProgram, public ParticlesProgramBase
{
public:
    LuminousParticlesProgram(GLSLSceneRenderer* renderer, const char* vertexShaderFile);

protected:
    virtual bool initializeRendering(SceneParticles* particles) override;
};
*/

}

#endif
