/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "SceneFountain.h"
#include <cnoid/GLSLSceneRenderer>
#include <cnoid/ShaderPrograms>
#include <cnoid/EigenUtil>
#include <memory>

using namespace std;
using namespace cnoid;

namespace {

float randFloat() {
    return ((float)rand() / RAND_MAX);
}

class FountainProgram : public NolightingProgram
{
public:
    FountainProgram(GLSLSceneRenderer* renderer);

    virtual void initialize() override;
    virtual void activate() override;
    virtual void initializeRendering() override;
    virtual void deactivate() override;

    void render(SceneFountain* fountain);

    GLSLSceneRenderer* renderer;
    bool isInitialized;
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
    gravity_ << 0.0f, 0.0f, -9.8f;
    angle_ = 0.1f;
}


FountainProgram::FountainProgram(GLSLSceneRenderer* renderer)
    : renderer(renderer)
{
    isInitialized = false;
}


void FountainProgram::initialize()
{
    if(ogl_LoadFunctions() == ogl_LOAD_FAILED){
        //return false;
    }
    
    loadVertexShader(":/PhenomenonPlugin/shader/fountain.vert");
    loadFragmentShader(":/PhenomenonPlugin/shader/fountain.frag");
    link();
    
    NolightingProgram::initialize();
    
    timeLocation = getUniformLocation("time");
    lifeTimeLocation = getUniformLocation("lifeTime");
    glUniform1f(lifeTimeLocation, 3.5f);

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
        phi = 2.0 * PI / 6.0f * randFloat();

        v.x() = sinf(theta) * cosf(phi);
        v.y() = cosf(theta);
        v.z() = sinf(theta) * sinf(phi);

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

    // Attach these to the torus's vertex array
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
}


void FountainProgram::activate()
{
    if(!isInitialized){
        initialize();
    }
    
    NolightingProgram::activate();

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
}
    

void FountainProgram::initializeRendering()
{

}


void FountainProgram::deactivate()
{
    glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);
}


void FountainProgram::render(SceneFountain* fountain)
{
    renderer->pushShaderProgram(*this, false);

    setProjectionMatrix(renderer->modelViewProjectionMatrix().cast<float>());
    
    glUniform1f(timeLocation, fountain->time());
    glBindVertexArray(vertexArray);
    glDrawArrays(GL_POINTS, 0, nParticles);

    fountain->setTime(fountain->time() + 0.01);

    renderer->popShaderProgram();
}
