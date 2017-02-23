/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PHENOMENON_PLUGIN_PARTICLES_PROGRAM_H
#define CNOID_PHENOMENON_PLUGIN_PARTICLES_PROGRAM_H

#include <cnoid/GLSLSceneRenderer>
#include <cnoid/ShaderPrograms>

namespace cnoid {

class ParticlesProgram : public LightingProgram
{
public:
    ParticlesProgram(GLSLSceneRenderer* renderer, const char* vertexShaderFile);
    void requestRendering(std::function<void()> renderingFunction);

protected:
    virtual bool initializeRendering();
    void setParticleTexture(const char* textureFile);
    void setTime(float time){
        glUniform1f(timeLocation, time);
    }
    static float random(float max = 1.0f) {
        return ((float)rand() / RAND_MAX) * max;
    }

private:
    enum State { NOT_INITIALIZED, INITIALIZED, FAILED } initializationState;
    GLSLSceneRenderer* renderer;
    std::string vertexShaderFile;
    GLint modelViewMatrixLocation;
    GLint projectionMatrixLocation;
    GLint pointSizeLocation;
    GLint angle2pixelsLocation;
    GLint timeLocation;
    GLint particleTexLocation;
    GLuint textureId;

    void render(const Matrix4f& MV, const std::function<void()>& renderingFunction);
};

}

#endif
