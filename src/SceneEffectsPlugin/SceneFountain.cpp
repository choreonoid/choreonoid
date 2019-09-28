/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "SceneEffectsPlugin.h"
#include "SceneFountain.h"
#include "ParticlesProgram.h"
#include <cnoid/EigenUtil>
#include <cnoid/GLSLProgram>

using namespace std;
using namespace cnoid;

namespace {

class FountainProgram : public ParticlesProgram
{
public:
    FountainProgram(GLSLSceneRenderer* renderer);
    virtual bool initializeRendering(SceneParticles* particles) override;
    void updateParticleBuffers(SceneFountain* fountain);
    void render(SceneFountain* fountain);

    GLint lifeTimeLocation;
    GLint accelLocation;

    // Variables to detect changes causing the buffer update
    int numParticles;
    float lifeTime;
    float emissionRange;
    float initialSpeedAverage;
    float initialSpeedVariation;

    GLuint& initVelBuffer;
    GLuint& offsetTimeBuffer;
    GLuint buffers[2];
    GLuint vertexArray;
};

struct Registration {
    Registration(){
        SgNode::registerType<SceneFountain, SceneParticles>();
        registerSceneEffectType<SceneFountain, FountainProgram>();
    }
} registration;


}


SceneFountain::SceneFountain()
    : SceneParticles(findPolymorphicId<SceneFountain>())
{
    setTexture(":/SceneEffectsPlugin/texture/bluewater.png");
}


SceneFountain::SceneFountain(const SceneFountain& org)
    : SceneParticles(org),
      particleSystem_(org.particleSystem_)
{

}


SgObject* SceneFountain::doClone(SgCloneMap*) const
{
    return new SceneFountain(*this);
}


ParticleSystem* SceneFountain::getParticleSystem()
{
    return &particleSystem_;
}


FountainProgram::FountainProgram(GLSLSceneRenderer* renderer)
    : ParticlesProgram(
        renderer,
        ":/SceneEffectsPlugin/shader/Fountain.vert",
        ":/SceneEffectsPlugin/shader/Particles.frag"),
      initVelBuffer(buffers[0]),
      offsetTimeBuffer(buffers[1])
{

}


bool FountainProgram::initializeRendering(SceneParticles* particles)
{
    if(!ParticlesProgramBase::initializeRendering(particles)){
        return false;
    }

    auto& glsl = glslProgram();
    lifeTimeLocation = glsl.getUniformLocation("lifeTime");
    accelLocation = glsl.getUniformLocation("accel");

    glGenBuffers(2, buffers);
    glGenVertexArrays(1, &vertexArray);

    SceneFountain* fountain = static_cast<SceneFountain*>(particles);
    updateParticleBuffers(fountain);

    return true;
}


void FountainProgram::updateParticleBuffers(SceneFountain* fountain)
{
    auto& ps = fountain->particleSystem();
    numParticles = ps.numParticles();
    emissionRange = ps.emissionRange();
    initialSpeedAverage = ps.initialSpeedAverage();
    initialSpeedVariation = ps.initialSpeedVariation();

    // Fill the first velocity buffer with random velocities
    Vector3f v;
    float speed, theta, phi;
    vector<GLfloat> data(numParticles * 3);
    for(GLuint i = 0; i < numParticles; ++i) {
        
        theta = emissionRange / 2.0f * frandom();
        phi = 2.0 * PI * frandom();

        v.x() = sinf(theta) * cosf(phi);
        v.y() = sinf(theta) * sinf(phi);
        v.z() = cosf(theta);

        speed = std::max(0.0f, initialSpeedAverage + initialSpeedVariation * (frandom() - 0.5f));
        v = v.normalized() * speed;

        data[3*i]   = v.x();
        data[3*i+1] = v.y();
        data[3*i+2] = v.z();
    }
    glBindBuffer(GL_ARRAY_BUFFER, initVelBuffer);
    glBufferData(GL_ARRAY_BUFFER, data.size() * sizeof(float), &data.front(), GL_STATIC_DRAW);
    
    // Fill the offset time buffer
    data.resize(numParticles);
    float rate = ps.lifeTime() / numParticles;
    float time = 0.0f;
    for(GLuint i = 0; i < numParticles; ++i) {
        data[i] = time;
        time += rate;
    }
    glBindBuffer(GL_ARRAY_BUFFER, offsetTimeBuffer);
    glBufferData(GL_ARRAY_BUFFER, data.size() * sizeof(float), &data.front(), GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindVertexArray(vertexArray);
    glBindBuffer(GL_ARRAY_BUFFER, initVelBuffer);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, offsetTimeBuffer);
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 0, NULL);
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
}


void FountainProgram::render(SceneFountain* fountain)
{
    auto& ps = fountain->particleSystem();

    if(!ps.on()){
        return;
    }
    
    if(numParticles != ps.numParticles() ||
       lifeTime != ps.lifeTime() ||
       emissionRange != ps.emissionRange() ||
       initialSpeedAverage != ps.initialSpeedAverage() ||
       initialSpeedVariation != ps.initialSpeedVariation()){
        updateParticleBuffers(fountain);
    }
    
    setTime(fountain->time() + ps.offsetTime());
    glUniform1f(lifeTimeLocation, ps.lifeTime());
    Vector3f accel = globalAttitude().transpose() * ps.acceleration();
    glUniform3fv(accelLocation, 1, accel.data());
    
    glBindVertexArray(vertexArray);
    glDrawArrays(GL_POINTS, 0, ps.numParticles());
}
