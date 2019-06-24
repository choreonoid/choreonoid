/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_SCENE_EFFECTS_PLUGIN_PARTICLES_PROGRAM_H
#define CNOID_SCENE_EFFECTS_PLUGIN_PARTICLES_PROGRAM_H

#include "SceneParticles.h"
#include <cnoid/ShaderPrograms>

namespace cnoid {

class GLSLSceneRenderer;

class ParticlesProgramBase
{
public:
    ParticlesProgramBase(GLSLSceneRenderer* renderer);
    
    void requestRendering(SceneParticles* particles, const std::function<void()>& renderingFunction);

    void setTime(float time){
        glUniform1f(timeLocation, time);
    }
    static float frandom(float max = 1.0f) {
        return ((float)rand() / RAND_MAX) * max;
    }

protected:
    virtual bool initializeRendering(SceneParticles* particles) = 0;
    virtual ShaderProgram* shaderProgram() = 0;
    GLSLSceneRenderer* renderer() { return renderer_; }
    const Matrix3f& globalAttitude() const { return globalAttitude_; }

private:
    enum State { NOT_INITIALIZED, INITIALIZED, FAILED } initializationState;
    GLSLSceneRenderer* renderer_;
    GLint modelViewMatrixLocation;
    GLint projectionMatrixLocation;
    GLint pointSizeLocation;
    GLint angle2pixelsLocation;
    GLint timeLocation;
    GLint particleTexLocation;
    GLuint textureId;
    Matrix3f globalAttitude_;

    void render(SceneParticles* particles, const Affine3& position, const std::function<void()>& renderingFunction);
};

    
class ParticlesProgram : public BasicLightingProgram, public ParticlesProgramBase
{
public:
    ParticlesProgram(
        GLSLSceneRenderer* renderer, const char* vertexShader, const char* fragmentShader);

protected:
    virtual ShaderProgram* shaderProgram() { return this; }
};


class LuminousParticlesProgram : public ShaderProgram, public ParticlesProgramBase
{
public:
    LuminousParticlesProgram(
        GLSLSceneRenderer* renderer, const char* vertexShader, const char* fragmentShader);

protected:
    virtual ShaderProgram* shaderProgram() { return this; }
};

}

#endif
