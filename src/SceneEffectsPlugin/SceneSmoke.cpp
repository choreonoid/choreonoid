/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "SceneEffectsPlugin.h"
#include "SceneSmoke.h"
#include "ParticlesProgram.h"
#include <cnoid/EigenUtil>

using namespace std;
using namespace cnoid;

namespace {

class SmokeProgram : public ParticlesProgram
{
public:
    SmokeProgram(GLSLSceneRenderer* renderer);
    virtual bool initializeRendering(SceneParticles* particles) override;
    void render(SceneSmoke* fountain);

    GLint lifeTimeLocation;
    GLuint nParticles;
    GLuint initVelBuffer;
    GLuint offsetTimeBuffer;
    GLuint vertexArray;
};

struct Registration {
    Registration(){
        SgNode::registerType<SceneSmoke, SceneParticles>();
        registerSceneEffectType<SceneSmoke, SmokeProgram>();
    }
} registration;


}


SceneSmoke::SceneSmoke()
    : SceneParticles(findPolymorphicId<SceneSmoke>())
{
    angle_ = 0.1f;
    lifeTime_ = 5.0f;
    acceleration_ << 0.0f, 0.0f, -9.8f;

    setParticleSize(0.06f);
    setTexture(":/SceneEffectsPlugin/texture/smoke.png");
}


SceneSmoke::SceneSmoke(const SceneSmoke& org)
    : SceneParticles(org)
{
    angle_ = org.angle_;
    lifeTime_ = org.lifeTime_;
    acceleration_ = org.acceleration_;
}


SgObject* SceneSmoke::clone(SgCloneMap& cloneMap) const
{
    return new SceneSmoke(*this);
}


SmokeProgram::SmokeProgram(GLSLSceneRenderer* renderer)
    : ParticlesProgram(renderer)
{

}


bool SmokeProgram::initializeRendering(SceneParticles* particles)
{
    loadVertexShader(":/SceneEffectsPlugin/shader/Smoke.vert");
    loadFragmentShader(":/SceneEffectsPlugin/shader/Particles.frag");
    link();
    
    if(!ParticlesProgramBase::initializeRendering(particles)){
        return false;
    }
    auto smoke = static_cast<SceneSmoke*>(particles);

    nParticles = 2000;

    // Fill the first velocity buffer with random velocities
    Vector3f v;
    float velocity, theta, phi;
    vector<GLfloat> data(nParticles * 3);
    for(GLuint i = 0; i < nParticles; ++i) {
        
        theta = PI / 3.0f * random();
        phi = 2.0 * PI * random();

        v.x() = sinf(theta) * cosf(phi);
        v.y() = sinf(theta) * sinf(phi);
        v.z() = cosf(theta);

        velocity = 0.1f + (0.2f - 0.1f) * random();
        v = v.normalized() * velocity;

        data[3*i]   = v.x();
        data[3*i+1] = v.y();
        data[3*i+2] = v.z();
    }
    glGenBuffers(1, &initVelBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, initVelBuffer);
    glBufferData(GL_ARRAY_BUFFER, data.size() * sizeof(float), &data.front(), GL_STATIC_DRAW);
    
    // Fill the offset time buffer
    data.resize(nParticles);
    float rate = smoke->lifeTime() / nParticles;
    float time = 0.0f;
    for(GLuint i = 0; i < nParticles; ++i) {
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

    return true;
}


void SmokeProgram::render(SceneSmoke* fountain)
{
    setTime(fountain->time());
    glUniform1f(lifeTimeLocation, fountain->lifeTime());
    glBindVertexArray(vertexArray);
    glDrawArrays(GL_POINTS, 0, nParticles);
}
