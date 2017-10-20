/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "SceneEffectsPlugin.h"
#include "SceneRainSnow.h"
#include "ParticlesProgram.h"
#include <cnoid/EigenUtil>

using namespace std;
using namespace cnoid;

namespace {

class RainSnowProgram : public ParticlesProgram
{
public:
    RainSnowProgram(GLSLSceneRenderer* renderer);
    virtual bool initializeRendering(SceneParticles* particles) override;
    void render(SceneRainSnowBase* particles);

    GLint velocityLocation;
    GLint lifeTimeLocation;

    GLfloat lifeTime;
    GLuint numParticles;
    GLuint initPosBuffer;
    GLuint offsetTimeBuffer;
    GLuint vertexArray;
};

struct Registration {
    Registration(){
        SgNode::registerType<SceneRainSnowBase, SceneParticles>();
        SgNode::registerType<SceneRain, SceneRainSnowBase>();
        SgNode::registerType<SceneSnow, SceneRainSnowBase>();

        registerSceneEffectType<SceneRain, RainSnowProgram>();
        registerSceneEffectType<SceneSnow, RainSnowProgram>();
    }
} registration;

}


SceneRainSnowBase::SceneRainSnowBase(int polymorphicId)
    : SceneParticles(polymorphicId)
{
    radius_ = 10.0f;
    top_ = 10.0f;
    bottom_ = 0.0f;
}


SceneRainSnowBase::SceneRainSnowBase(const SceneRainSnowBase& org)
    : SceneParticles(org)
{
    radius_ = org.radius_;
    top_ = org.top_;
    bottom_ = org.bottom_;
    velocity_ = org.velocity_;
}


SceneRain::SceneRain()
    : SceneRainSnowBase(findPolymorphicId<SceneRain>())
{
    setVelocity(Vector3f(0.0f, 0.0f, -5.0f));
    setParticleSize(0.02f);
    setTexture(":/SceneEffectsPlugin/texture/rain.png");
}


SceneRain::SceneRain(const SceneRain& org)
    : SceneRainSnowBase(org)
{

}


SgObject* SceneRain::clone(SgCloneMap& cloneMap) const
{
    return new SceneRain(*this);
}


SceneSnow::SceneSnow()
    : SceneRainSnowBase(findPolymorphicId<SceneSnow>())
{
    setVelocity(Vector3f(0.0f, 0.0f, -0.3f));
    setParticleSize(0.025f);
    setTexture(":/SceneEffectsPlugin/texture/snow.png");
}


SceneSnow::SceneSnow(const SceneSnow& org)
    : SceneRainSnowBase(org)
{

}


SgObject* SceneSnow::clone(SgCloneMap& cloneMap) const
{
    return new SceneSnow(*this);
}



RainSnowProgram::RainSnowProgram(GLSLSceneRenderer* renderer)
    : ParticlesProgram(renderer)
{

}


bool RainSnowProgram::initializeRendering(SceneParticles* particles)
{
    loadVertexShader(":/SceneEffectsPlugin/shader/RainSnow.vert");
    loadFragmentShader(":/SceneEffectsPlugin/shader/Particles.frag");
    link();
    
    if(!ParticlesProgramBase::initializeRendering(particles)){
        return false;
    }

    numParticles = 8000;
    SceneSnow* snow = static_cast<SceneSnow*>(particles);

    // Initial position buffer
    vector<GLfloat> data(numParticles * 3);
    const float r = snow->radius();
    const float r2 = r * 4;
    for(GLuint i = 0; i < numParticles; ++i) {
        float x, y;
        while(true){
            x = 2.0 * r * random() - r;
            y = 2.0 * r * random() - r;
            if(x * x + y * y <= r2){
                break;
            }
        }
        data[3*i]     = x;
        data[3*i + 1] = y;
        data[3*i + 2] = snow->top();
    }
    glGenBuffers(1, &initPosBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, initPosBuffer);
    glBufferData(GL_ARRAY_BUFFER, data.size() * sizeof(float), &data.front(), GL_STATIC_DRAW);

    // Offset time buffer
    data.resize(numParticles);
    lifeTime = fabsf((snow->top() - snow->bottom()) / snow->velocity().z());
    float rate = lifeTime / numParticles;
    float time = 0.0f;
    for(GLuint i = 0; i < numParticles; ++i) {
        data[i] = time;
        time += rate;
    }
    glGenBuffers(1, &offsetTimeBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, offsetTimeBuffer);
    glBufferData(GL_ARRAY_BUFFER, data.size() * sizeof(float), &data.front(), GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // Vertex arrays
    glGenVertexArrays(1, &vertexArray);
    glBindVertexArray(vertexArray);
    glBindBuffer(GL_ARRAY_BUFFER, initPosBuffer);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, offsetTimeBuffer);
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 0, NULL);
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);

    velocityLocation = getUniformLocation("velocity");
    lifeTimeLocation = getUniformLocation("lifeTime");

    return true;
}


void RainSnowProgram::render(SceneRainSnowBase* particles)
{
    setTime(particles->time());

    glUniform1f(lifeTimeLocation, lifeTime);
    glUniform3fv(velocityLocation, 1, particles->velocity().data());

    glBindVertexArray(vertexArray);
    glDrawArrays(GL_POINTS, 0, numParticles);
}
