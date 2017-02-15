/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "SceneFountain.h"
#include <cnoid/GLSLSceneRenderer>
#include <cnoid/ShaderPrograms>
#include <cnoid/EigenUtil>
#include <memory>
#include <iostream>

using namespace std;
using namespace cnoid;

namespace {

float randFloat() {
    return ((float)rand() / RAND_MAX);
}

class FountainProgram : public ShaderProgram
{
public:
    FountainProgram(GLSLSceneRenderer* renderer);
    bool initialize();
    void render(SceneFountain* fountain);
    void doRender(SceneFountain* fountain);

    GLSLSceneRenderer* renderer;
    bool isInitialized;
    GLint MVPLocation;
    GLint timeLocation;
    GLint lifeTimeLocation;
    GLuint nParticles;
    GLuint initVelBuffer;
    GLuint startTimeBuffer;
    GLuint vertexArray;
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


bool FountainProgram::initialize()
{
    if(ogl_LoadFunctions() == ogl_LOAD_FAILED){
        cout << "ogl_LoadFunctions() == ogl_LOAD_FAILED" << endl;
        return false;
    }
    
    loadVertexShader(":/PhenomenonPlugin/shader/fountain.vert");
    loadFragmentShader(":/PhenomenonPlugin/shader/fountain.frag");
    link();
    
    MVPLocation = getUniformLocation("MVP");
    timeLocation = getUniformLocation("time");
    lifeTimeLocation = getUniformLocation("lifeTime");

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

    isInitialized = true;

    return true;
}


void FountainProgram::render(SceneFountain* fountain)
{
    if(renderer->isPicking()){
        return;
    }

    if(!isInitialized){
        if(!initialize()){
            renderer->renderingFunctions().resetFunction<SceneFountain>(true);
            return;
        }
    }

    renderer->dispatchToTransparentPhase([this, fountain](){ doRender(fountain); });
}


void FountainProgram::doRender(SceneFountain* fountain)
{
    renderer->pushShaderProgram(*this, false);

    const Matrix4f M = renderer->modelViewProjectionMatrix().cast<float>();
    glUniformMatrix4fv(MVPLocation, 1, GL_FALSE, M.data());
    
    glUniform1f(timeLocation, fountain->time());
    glUniform1f(lifeTimeLocation, fountain->lifeTime());
    glBindVertexArray(vertexArray);
    glDrawArrays(GL_POINTS, 0, nParticles);

    renderer->popShaderProgram();

    fountain->setTime(fountain->time() + 0.005);
}
