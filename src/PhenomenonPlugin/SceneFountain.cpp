/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "SceneFountain.h"
#include <cnoid/GLSLSceneRenderer>
#include <cnoid/ShaderPrograms>
#include <cnoid/EigenUtil>
#include <QImage>
#include <memory>
#include <iostream>

using namespace std;
using namespace cnoid;

namespace {

float randFloat() {
    return ((float)rand() / RAND_MAX);
}

class FountainProgram : public LightingProgram
{
public:
    FountainProgram(GLSLSceneRenderer* renderer);
    bool initializeRendering();
    void render(SceneFountain* fountain);
    void doRender(SceneFountain* fountain, const Matrix4f& MV);

    GLSLSceneRenderer* renderer;
    bool isInitialized;
    GLint modelViewMatrixLocation;
    GLint projectionMatrixLocation;
    GLint pointSizeLocation;
    GLint angle2pixelsLocation;
    GLint timeLocation;
    GLint lifeTimeLocation;
    GLint cycleTimeLocation;
    GLint particleTexLocation;
    GLuint nParticles;
    GLuint initVelBuffer;
    GLuint startTimeBuffer;
    GLuint vertexArray;
    GLuint textureId;
};

}


void SceneFountain::initializeClass()
{
    SgNode::registerType<SceneFountain, SgNode>();

    GLSLSceneRenderer::addExtension(
        [](GLSLSceneRenderer* renderer){
            shared_ptr<FountainProgram> program = make_shared<FountainProgram>(renderer);
            renderer->renderingFunctions().setFunction<SceneFountain>(
                [program](SceneFountain* fountain){
                    program->render(fountain);
                });
        });
}


SceneFountain::SceneFountain()
    : SgNode(findPolymorphicId<SceneFountain>())
{
    time_ = 0.0f;
    lifeTime_ = 4.0f;
    gravity_ << 0.0f, 0.0f, -9.8f;
    angle_ = 0.1f;
}


FountainProgram::FountainProgram(GLSLSceneRenderer* renderer)
    : renderer(renderer)
{
    isInitialized = false;
}


bool FountainProgram::initializeRendering()
{
    if(ogl_LoadFunctions() == ogl_LOAD_FAILED){
        cout << "ogl_LoadFunctions() == ogl_LOAD_FAILED" << endl;
        return false;
    }
    
    loadVertexShader(":/PhenomenonPlugin/shader/fountain.vert");
    loadFragmentShader(":/PhenomenonPlugin/shader/particles.frag");
    link();

    LightingProgram::initialize();

    modelViewMatrixLocation = getUniformLocation("modelViewMatrix");
    projectionMatrixLocation = getUniformLocation("projectionMatrix");
    pointSizeLocation = getUniformLocation("pointSize");
    angle2pixelsLocation = getUniformLocation("angle2pixels");
    
    nParticles = 8000;

    // Generate the buffers
    glGenBuffers(1, &initVelBuffer);   // Initial velocity buffer
    glGenBuffers(1, &startTimeBuffer); // Start time buffer

    // Allocate space for all buffers
    int size = nParticles * 3 * sizeof(float);
    glBindBuffer(GL_ARRAY_BUFFER, initVelBuffer);
    glBufferData(GL_ARRAY_BUFFER, size, NULL, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, startTimeBuffer);
    glBufferData(GL_ARRAY_BUFFER, nParticles * sizeof(float), NULL, GL_STATIC_DRAW);

    // Fill the first velocity buffer with random velocities
    Vector3f v;
    float velocity, theta, phi;
    vector<GLfloat> data(nParticles * 3);
    for(int i = 0; i < nParticles; ++i) {
        
        theta = PI / 6.0f * randFloat();
        phi = 2.0 * PI * randFloat();

        v.x() = sinf(theta) * cosf(phi);
        v.y() = sinf(theta) * sinf(phi);
        v.z() = cosf(theta);

        velocity = 1.24f + (1.5f - 1.25f) * randFloat();
        v = v.normalized() * velocity;

        data[3*i]   = v.x();
        data[3*i+1] = v.y();
        data[3*i+2] = v.z();
    }
    glBindBuffer(GL_ARRAY_BUFFER, initVelBuffer);
    glBufferSubData(GL_ARRAY_BUFFER, 0, size, &data.front());

    // Fill the start time buffer
    data.resize(nParticles);
    float time = 0.0f;
    float rate = 0.00075f;
    for(int i = 0; i < nParticles; ++i) {
        data[i] = time;
        time += rate;
    }
    glBindBuffer(GL_ARRAY_BUFFER, startTimeBuffer);
    glBufferSubData(GL_ARRAY_BUFFER, 0, nParticles * sizeof(float), &data.front());

    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glGenVertexArrays(1, &vertexArray);
    glBindVertexArray(vertexArray);
    glBindBuffer(GL_ARRAY_BUFFER, initVelBuffer);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, startTimeBuffer);
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 0, NULL);
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);

    QImage image(":/PhenomenonPlugin/texture/bluewater.png");
    QImage texture = image.rgbSwapped();
    glActiveTexture(GL_TEXTURE0);
    glGenTextures(1, &textureId);
    glBindTexture(GL_TEXTURE_2D, textureId);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, texture.width(), texture.height(),
                 0, GL_RGBA, GL_UNSIGNED_BYTE, texture.constBits());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    timeLocation = getUniformLocation("time");
    lifeTimeLocation = getUniformLocation("lifeTime");
    cycleTimeLocation = getUniformLocation("cycleTime");
    particleTexLocation = getUniformLocation("particleTex");

    isInitialized = true;

    return true;
}


void FountainProgram::render(SceneFountain* fountain)
{
    if(renderer->isPicking()){
        return;
    }

    if(!isInitialized){
        if(!initializeRendering()){
            renderer->renderingFunctions().resetFunction<SceneFountain>(true);
            return;
        }
    }

    const Matrix4f MV = renderer->modelViewMatrix().cast<float>();
    renderer->dispatchToTransparentPhase([this, fountain, MV](){ doRender(fountain, MV); });
}


void FountainProgram::doRender(SceneFountain* fountain, const Matrix4f& MV)
{
    renderer->pushShaderProgram(*this, false);

    renderer->renderLights(this);
    renderer->renderFog(this);

    glUniformMatrix4fv(modelViewMatrixLocation, 1, GL_FALSE, MV.data());
    const Matrix4f P = renderer->projectionMatrix().cast<float>();
    glUniformMatrix4fv(projectionMatrixLocation, 1, GL_FALSE, P.data());
    glUniform1f(pointSizeLocation, 0.08f);
    
    SgPerspectiveCamera* camera = dynamic_cast<SgPerspectiveCamera*>(renderer->currentCamera());
    if(camera){
        int x, y, width, height;
        renderer->getViewport(x, y, width, height);
        glUniform1f(angle2pixelsLocation, height / camera->fovy((double)width / height));
    }
    
    glUniform1f(timeLocation, fountain->time());
    glUniform1f(lifeTimeLocation, fountain->lifeTime());
    glUniform1f(cycleTimeLocation, 6.0f);
    glUniform1i(particleTexLocation, 0);
    
    glBindVertexArray(vertexArray);
    glDrawArrays(GL_POINTS, 0, nParticles);

    renderer->popShaderProgram();
}
