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

    /*
    template<class SceneNodeType, class SceneNodeBaseType, class ShaderProgramType>
    struct Registration
    {
        Registration(){

            GLSLSceneRenderer::addExtension(
                [](GLSLSceneRenderer* renderer){
                    std::shared_ptr<ShaderProgramType> program = std::make_shared<ShaderProgramType>(renderer);
                    renderer->renderingFunctions().setFunction<SceneNodeType>(
                        [program](SceneNodeType* particles){
                            program->requestRendering(particles, [program, particles]() { program->render(particles); });
                        });
                });
        }
    };
    */

    template<class SceneNode, class Program>
    static void registerType(){
        GLSLSceneRenderer::addExtension(
            [](GLSLSceneRenderer* renderer){
                auto program = std::make_shared<Program>(renderer);
                renderer->renderingFunctions().setFunction<SceneNode>(
                    [program](SceneNode* particles){
                        program->requestRendering(particles, [program, particles]() { program->render(particles); });
                    });
            });
    }
    
    ParticlesProgram(GLSLSceneRenderer* renderer, const char* vertexShaderFile);
    void requestRendering(SceneParticles* particles, std::function<void()> renderingFunction);

protected:
    virtual bool initializeRendering(SceneParticles* particles);

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
