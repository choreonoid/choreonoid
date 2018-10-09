/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "SceneEffectsPlugin.h"
#include "SceneFountain.h"
#include "ParticlesProgram.h"
#include <cnoid/EigenUtil>

using namespace std;
using namespace cnoid;

namespace {

class FountainProgram : public ParticlesProgram
{
public:
    FountainProgram(GLSLSceneRenderer* renderer);
    virtual bool initializeRendering(SceneParticles* particles) override;
    void render(SceneFountain* fountain);

    GLint lifeTimeLocation;
    GLint accelLocation;
    GLuint nParticles;
    GLuint initVelBuffer;
    GLuint offsetTimeBuffer;
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


SgObject* SceneFountain::clone(SgCloneMap& cloneMap) const
{
    return new SceneFountain(*this);
}


ParticleSystem* SceneFountain::getParticleSystem()
{
    return &particleSystem_;
}


FountainProgram::FountainProgram(GLSLSceneRenderer* renderer)
    : ParticlesProgram(renderer)
{

}


bool FountainProgram::initializeRendering(SceneParticles* particles)
{
    loadVertexShader(":/SceneEffectsPlugin/shader/Fountain.vert");
    loadFragmentShader(":/SceneEffectsPlugin/shader/Particles.frag");
    link();
    
    if(!ParticlesProgramBase::initializeRendering(particles)){
        return false;
    }

    auto ps = particles->getParticleSystem();

    // Fill the first velocity buffer with random velocities
    Vector3f v;
    float velocity, theta, phi;
    vector<GLfloat> data(ps->numParticles() * 3);
    for(GLuint i = 0; i < ps->numParticles(); ++i) {
        
        theta = ps->emissionRange() / 2.0f * random();
        phi = 2.0 * PI * random();

        v.x() = sinf(theta) * cosf(phi);
        v.y() = sinf(theta) * sinf(phi);
        v.z() = cosf(theta);

        velocity = 1.25f + (1.5f - 1.25f) * random();
        v = v.normalized() * velocity;

        data[3*i]   = v.x();
        data[3*i+1] = v.y();
        data[3*i+2] = v.z();
    }
    glGenBuffers(1, &initVelBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, initVelBuffer);
    glBufferData(GL_ARRAY_BUFFER, data.size() * sizeof(float), &data.front(), GL_STATIC_DRAW);
    
    // Fill the offset time buffer
    data.resize(ps->numParticles());
    float rate = ps->lifeTime() / ps->numParticles();
    float time = 0.0f;
    for(GLuint i = 0; i < ps->numParticles(); ++i) {
        data[i] = time;
        time += rate;
    }
    glGenBuffers(1, &offsetTimeBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, offsetTimeBuffer);
    glBufferData(GL_ARRAY_BUFFER, data.size() * sizeof(float), &data.front(), GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glGenVertexArrays(1, &vertexArray);
    glBindVertexArray(vertexArray);
    glBindBuffer(GL_ARRAY_BUFFER, initVelBuffer);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, offsetTimeBuffer);
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 0, NULL);
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);

    lifeTimeLocation = getUniformLocation("lifeTime");
    accelLocation = getUniformLocation("accel");

    return true;
}


void FountainProgram::render(SceneFountain* fountain)
{
    auto& ps = fountain->particleSystem();
    if(ps.on()){
        setTime(fountain->time() + ps.offsetTime());
        glUniform1f(lifeTimeLocation, ps.lifeTime());
        Vector3f accel = globalAttitude().transpose() * ps.acceleration();
        glUniform3fv(accelLocation, 1, accel.data());
        glBindVertexArray(vertexArray);
        glDrawArrays(GL_POINTS, 0, ps.numParticles());
    }
}
