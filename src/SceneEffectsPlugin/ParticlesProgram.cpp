/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "ParticlesProgram.h"
#include <cnoid/EigenUtil>
#include <QImage>
#include <memory>
#include <mutex>

using namespace std;
using namespace cnoid;


ParticlesProgramBase::ParticlesProgramBase(GLSLSceneRenderer* renderer)
    : renderer_(renderer)
{
    
    initializationState = NOT_INITIALIZED;
}


bool ParticlesProgramBase::initializeRendering(SceneParticles* particles)
{
    auto program = shaderProgram();

    program->initialize();

    modelViewMatrixLocation = program->getUniformLocation("modelViewMatrix");
    projectionMatrixLocation = program->getUniformLocation("projectionMatrix");
    pointSizeLocation = program->getUniformLocation("pointSize");
    angle2pixelsLocation = program->getUniformLocation("angle2pixels");
    
    timeLocation = program->getUniformLocation("time");
    particleTexLocation = program->getUniformLocation("particleTex");

    if(!particles->texture().empty()){
        QImage image(particles->texture().c_str());
        QImage texture = image.rgbSwapped();
        glActiveTexture(GL_TEXTURE0);
        glGenTextures(1, &textureId);
        glBindTexture(GL_TEXTURE_2D, textureId);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, texture.width(), texture.height(),
                     0, GL_RGBA, GL_UNSIGNED_BYTE, texture.constBits());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    }

    return true;
}


void ParticlesProgramBase::requestRendering(SceneParticles* particles, std::function<void()> renderingFunction)
{
    if(renderer_->isPicking()){
        return;
    }

    if(initializationState != INITIALIZED){
        if(initializationState == FAILED){
            return;
        } else {
            {
                static std::mutex mutex;
                static bool isOglFunctionsLoaded = false;
                std::lock_guard<std::mutex> guard(mutex);
                if(!isOglFunctionsLoaded){
                    if(ogl_LoadFunctions() == ogl_LOAD_FAILED){
                        initializationState = FAILED;
                        return;
                    }
                    isOglFunctionsLoaded = true;
                }
            }
            if(initializeRendering(particles)){
                initializationState = INITIALIZED;
            } else {
                initializationState = FAILED;
                return;
            }
        }
    }

    Matrix3f R = renderer_->currentModelTransform().linear().cast<float>();
    const Matrix4f MV = renderer_->modelViewMatrix().cast<float>();
    renderer_->dispatchToTransparentPhase([=](){ render(particles, R, MV, renderingFunction); });
}


void ParticlesProgramBase::render
(SceneParticles* particles, const Matrix3f& R, const Matrix4f& MV, const std::function<void()>& renderingFunction)
{
    ShaderProgram* program = shaderProgram();
    
    renderer_->pushShaderProgram(*program, false);

    auto lightingProgram = dynamic_cast<LightingProgram*>(program);
    if(lightingProgram){
        renderer_->renderLights(lightingProgram);
        renderer_->renderFog(lightingProgram);
    }

    glUniformMatrix4fv(modelViewMatrixLocation, 1, GL_FALSE, MV.data());
    const Matrix4f P = renderer_->projectionMatrix().cast<float>();
    glUniformMatrix4fv(projectionMatrixLocation, 1, GL_FALSE, P.data());

    int x, y, width, height;
    renderer_->getViewport(x, y, width, height);
    SgCamera* camera = renderer_->currentCamera();
    auto ps = particles->getParticleSystem();
    
    if(SgPerspectiveCamera* persCamera = dynamic_cast<SgPerspectiveCamera*>(camera)){
        glUniform1f(angle2pixelsLocation, height / persCamera->fovy((double)width / height));
        glUniform1f(pointSizeLocation, ps->particleSize());
    } else if(SgOrthographicCamera* orthoCamera = dynamic_cast<SgOrthographicCamera*>(camera)){
        //float size = 0.08f * height / orthoCamera->height();
        glUniform1f(pointSizeLocation, -ps->particleSize());
    }

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, textureId);
    glBindSampler(0, 0);
    
    glUniform1i(particleTexLocation, 0);

    globalAttitude_ = R;
    renderingFunction();

    renderer_->popShaderProgram();
}


ParticlesProgram::ParticlesProgram(GLSLSceneRenderer* renderer)
    : ParticlesProgramBase(renderer)
{

}


LuminousParticlesProgram::LuminousParticlesProgram(GLSLSceneRenderer* renderer)
    : ParticlesProgramBase(renderer)
{

}
