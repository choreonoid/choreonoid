/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "SceneEffectsPlugin.h"
#include "SceneRainSnow.h"
#include "ParticlesProgram.h"
#include <cnoid/EigenUtil>
#include <cnoid/GLSLProgram>

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


ParticleSystem* SceneRainSnowBase::getParticleSystem()
{
    return &particleSystem_;
}


SceneRain::SceneRain()
    : SceneRainSnowBase(findPolymorphicId<SceneRain>())
{
    setVelocity(Vector3f(0.0f, 0.0f, -5.0f));
    setTexture(":/SceneEffectsPlugin/texture/rain.png");
}


SceneRain::SceneRain(const SceneRain& org)
    : SceneRainSnowBase(org)
{

}


SgObject* SceneRain::doClone(SgCloneMap*) const
{
    return new SceneRain(*this);
}


SceneSnow::SceneSnow()
    : SceneRainSnowBase(findPolymorphicId<SceneSnow>())
{
    setVelocity(Vector3f(0.0f, 0.0f, -0.3f));
    setTexture(":/SceneEffectsPlugin/texture/snow.png");
}


SceneSnow::SceneSnow(const SceneSnow& org)
    : SceneRainSnowBase(org)
{

}


SgObject* SceneSnow::doClone(SgCloneMap*) const
{
    return new SceneSnow(*this);
}



RainSnowProgram::RainSnowProgram(GLSLSceneRenderer* renderer)
    : ParticlesProgram(
        renderer,
        ":/SceneEffectsPlugin/shader/RainSnow.vert",
        ":/SceneEffectsPlugin/shader/Particles.frag")
{

}


bool RainSnowProgram::initializeRendering(SceneParticles* particles)
{
    if(!ParticlesProgramBase::initializeRendering(particles)){
        return false;
    }

    auto rs = static_cast<SceneRainSnowBase*>(particles);
    auto& ps = rs->particleSystem();

    // Initial position buffer
    vector<GLfloat> data(ps.numParticles() * 3);
    const float r = rs->radius();
    const float r2 = r * 4;
    for(GLuint i = 0; i < ps.numParticles(); ++i) {
        float x, y;
        while(true){
            x = 2.0 * r * frandom() - r;
            y = 2.0 * r * frandom() - r;
            if(x * x + y * y <= r2){
                break;
            }
        }
        data[3*i]     = x;
        data[3*i + 1] = y;
        data[3*i + 2] = rs->top();
    }
    glGenBuffers(1, &initPosBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, initPosBuffer);
    glBufferData(GL_ARRAY_BUFFER, data.size() * sizeof(float), &data.front(), GL_STATIC_DRAW);

    // Offset time buffer
    data.resize(ps.numParticles());
    lifeTime = fabsf((rs->top() - rs->bottom()) / rs->velocity().z());
    float rate = lifeTime / ps.numParticles();
    float time = 0.0f;
    for(GLuint i = 0; i < ps.numParticles(); ++i) {
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

    auto& glsl = glslProgram();
    velocityLocation = glsl.getUniformLocation("velocity");
    lifeTimeLocation = glsl.getUniformLocation("lifeTime");

    return true;
}


void RainSnowProgram::render(SceneRainSnowBase* particles)
{
    auto& ps = particles->particleSystem();

    setTime(particles->time() + ps.offsetTime());

    glUniform1f(lifeTimeLocation, lifeTime);
    glUniform3fv(velocityLocation, 1, particles->velocity().data());

    glBindVertexArray(vertexArray);
    glDrawArrays(GL_POINTS, 0, ps.numParticles());
}
