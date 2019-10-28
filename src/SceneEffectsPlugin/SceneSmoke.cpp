/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "SceneEffectsPlugin.h"
#include "SceneSmoke.h"
#include "ParticlesProgram.h"
#include <cnoid/EigenUtil>
#include <cnoid/GLSLProgram>

using namespace std;
using namespace cnoid;

namespace {

class SmokeProgram : public ParticlesProgram
{
public:
    SmokeProgram(GLSLSceneRenderer* renderer);
    virtual bool initializeRendering(SceneParticles* particles) override;
    void updateParticleBuffers(SceneSmoke* smoke);
    void render(SceneSmoke* fountain);

    GLint lifeTimeLocation;
    GLint accelLocation;

    // Variables to detect changes causing the buffer update
    int numParticles;
    float lifeTime;
    float emissionRange;

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
    setTexture(":/SceneEffectsPlugin/texture/smoke.png");
}


SceneSmoke::SceneSmoke(const SceneSmoke& org)
    : SceneParticles(org),
      particleSystem_(org.particleSystem_)
{

}


Referenced* SceneSmoke::doClone(CloneMap*) const
{
    return new SceneSmoke(*this);
}


ParticleSystem* SceneSmoke::getParticleSystem()
{
    return &particleSystem_;
}


SmokeProgram::SmokeProgram(GLSLSceneRenderer* renderer)
    : ParticlesProgram(
        renderer,
        ":/SceneEffectsPlugin/shader/Smoke.vert",
        ":/SceneEffectsPlugin/shader/Particles.frag")
{

}


bool SmokeProgram::initializeRendering(SceneParticles* particles)
{
    if(!ParticlesProgramBase::initializeRendering(particles)){
        return false;
    }

    auto& glsl = glslProgram();
    lifeTimeLocation = glsl.getUniformLocation("lifeTime");
    accelLocation = glsl.getUniformLocation("accel");

    glGenBuffers(1, &initVelBuffer);
    glGenBuffers(1, &offsetTimeBuffer);
    glGenVertexArrays(1, &vertexArray);

    SceneSmoke* smoke = static_cast<SceneSmoke*>(particles);
    updateParticleBuffers(smoke);

    return true;
}


void SmokeProgram::updateParticleBuffers(SceneSmoke* smoke)
{
    auto& ps = smoke->particleSystem();
    numParticles = ps.numParticles();
    lifeTime = ps.lifeTime();
    emissionRange = ps.emissionRange();
    
    // Fill the first velocity buffer with random velocities
    Vector3f v;
    float velocity, theta, phi;
    vector<GLfloat> data(numParticles * 3);
    srand(0);
    for(GLuint i = 0; i < numParticles; ++i) {
        theta = emissionRange / 2.0f * frandom();
        phi = 2.0 * PI * frandom();

        v.x() = sinf(theta) * cosf(phi);
        v.y() = sinf(theta) * sinf(phi);
        v.z() = cosf(theta);

        velocity = 0.1f + (0.2f - 0.1f) * frandom();
        v = v.normalized() * velocity;

        data[3*i]   = v.x();
        data[3*i+1] = v.y();
        data[3*i+2] = v.z();
    }

    glBindBuffer(GL_ARRAY_BUFFER, initVelBuffer);
    glBufferData(GL_ARRAY_BUFFER, data.size() * sizeof(float), &data.front(), GL_STATIC_DRAW);
    
    // Fill the offset time buffer
    data.resize(numParticles);
    float rate = lifeTime / numParticles;
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


void SmokeProgram::render(SceneSmoke* smoke)
{
    auto& ps = smoke->particleSystem();

    if(!ps.on()){
        return;
    }

    if(numParticles != ps.numParticles() ||
       lifeTime != ps.lifeTime() ||
       emissionRange != ps.emissionRange()){
        updateParticleBuffers(smoke);
    }
    
    setTime(smoke->time() + ps.offsetTime());
    glUniform1f(lifeTimeLocation, ps.lifeTime());
    Vector3f accel = globalAttitude().transpose() * ps.acceleration();
    glUniform3fv(accelLocation, 1, accel.data());

    /*
    GLint blendSrc, blendDst;
    glGetIntegerv(GL_BLEND_SRC_ALPHA, &blendSrc);
    glGetIntegerv(GL_BLEND_DST_ALPHA, &blendDst);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    */
    
    glBindVertexArray(vertexArray);
    glDrawArrays(GL_POINTS, 0, ps.numParticles());

    //glBlendFunc(blendSrc, blendDst);
}
