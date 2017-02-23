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

class ParticlesProgram : public LightingProgram
{
public:

    template<class Program>
    struct Registration
    {
        Registration(){
            SgNode::registerType<typename Program::NodeType, SceneParticles>();

            GLSLSceneRenderer::addExtension(
                [](GLSLSceneRenderer* renderer){
                    std::shared_ptr<Program> program = std::make_shared<Program>(renderer);
                    renderer->renderingFunctions().setFunction<typename Program::NodeType>(
                        [program](typename Program::NodeType* particles){
                            program->requestRendering(particles, [program, particles]() { program->render(particles); });
                        });
                });
        }
    };
    
    ParticlesProgram(GLSLSceneRenderer* renderer, const char* vertexShaderFile);
    void requestRendering(SceneParticles* particles, std::function<void()> renderingFunction);

protected:
    virtual bool initializeRendering(SceneParticles* particles);
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

    void render(SceneParticles* particles, const Matrix4f& MV, const std::function<void()>& renderingFunction);
};

}

#endif
