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


ParticlesProgram::ParticlesProgram(GLSLSceneRenderer* renderer, const char* vertexShaderFile)
    : renderer(renderer),
      vertexShaderFile(vertexShaderFile)
{
    
    initializationState = NOT_INITIALIZED;
}


bool ParticlesProgram::initializeRendering(SceneParticles* particles)
{
    loadVertexShader(vertexShaderFile.c_str());
    loadFragmentShader(":/PhenomenonPlugin/shader/particles.frag");
    link();

    LightingProgram::initialize();

    modelViewMatrixLocation = getUniformLocation("modelViewMatrix");
    projectionMatrixLocation = getUniformLocation("projectionMatrix");
    pointSizeLocation = getUniformLocation("pointSize");
    angle2pixelsLocation = getUniformLocation("angle2pixels");
    
    timeLocation = getUniformLocation("time");
    particleTexLocation = getUniformLocation("particleTex");

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


void ParticlesProgram::requestRendering(SceneParticles* particles, std::function<void()> renderingFunction)
{
    if(renderer->isPicking()){
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

    const Matrix4f MV = renderer->modelViewMatrix().cast<float>();
    renderer->dispatchToTransparentPhase([=](){ render(particles, MV, renderingFunction); });
}


void ParticlesProgram::render(SceneParticles* particles, const Matrix4f& MV, const std::function<void()>& renderingFunction)
{
    renderer->pushShaderProgram(*this, false);

    renderer->renderLights(this);
    renderer->renderFog(this);

    glUniformMatrix4fv(modelViewMatrixLocation, 1, GL_FALSE, MV.data());
    const Matrix4f P = renderer->projectionMatrix().cast<float>();
    glUniformMatrix4fv(projectionMatrixLocation, 1, GL_FALSE, P.data());

    int x, y, width, height;
    renderer->getViewport(x, y, width, height);
    SgCamera* camera = renderer->currentCamera();
    if(SgPerspectiveCamera* persCamera = dynamic_cast<SgPerspectiveCamera*>(camera)){
        glUniform1f(angle2pixelsLocation, height / persCamera->fovy((double)width / height));
        glUniform1f(pointSizeLocation, particles->particleSize());
    } else if(SgOrthographicCamera* orthoCamera = dynamic_cast<SgOrthographicCamera*>(camera)){
        float size = 0.08f * height / orthoCamera->height();
        glUniform1f(pointSizeLocation, -particles->particleSize());
    }

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, textureId);
    
    glUniform1i(particleTexLocation, 0);

    renderingFunction();

    renderer->popShaderProgram();
}
