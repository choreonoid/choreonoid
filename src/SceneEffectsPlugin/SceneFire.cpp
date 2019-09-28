/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "SceneEffectsPlugin.h"
#include "SceneFire.h"
#include "ParticlesProgram.h"
#include <cnoid/EigenUtil>
#include <cnoid/GLSLProgram>

using namespace std;
using namespace cnoid;

namespace {

class FireProgram : public LuminousParticlesProgram
{
public:
    FireProgram(GLSLSceneRenderer* renderer);
    virtual bool initializeRendering(SceneParticles* particles) override;
    void updateParticleBuffers(SceneFire* fire);
    void render(SceneFire* fountain);

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
        SgNode::registerType<SceneFire, SceneParticles>();
        registerSceneEffectType<SceneFire, FireProgram>();
    }
} registration;

}


SceneFire::SceneFire()
    : SceneParticles(findPolymorphicId<SceneFire>())
{
    setTexture(":/SceneEffectsPlugin/texture/fire.png");
}


SceneFire::SceneFire(const SceneFire& org)
    : SceneParticles(org),
      particleSystem_(org.particleSystem_)
{

}


SgObject* SceneFire::doClone(SgCloneMap*) const
{
    return new SceneFire(*this);
}


ParticleSystem* SceneFire::getParticleSystem()
{
    return &particleSystem_;
}


FireProgram::FireProgram(GLSLSceneRenderer* renderer)
    : LuminousParticlesProgram(
        renderer,
        ":/SceneEffectsPlugin/shader/Fire.vert",
        ":/SceneEffectsPlugin/shader/LuminousParticles.frag"),
      initVelBuffer(buffers[0]),
      offsetTimeBuffer(buffers[1])
{

}


bool FireProgram::initializeRendering(SceneParticles* particles)
{
    if(!ParticlesProgramBase::initializeRendering(particles)){
        return false;
    }

    auto& glsl = glslProgram();
    lifeTimeLocation = glsl.getUniformLocation("lifeTime");
    accelLocation = glsl.getUniformLocation("accel");

    glGenBuffers(2, buffers);
    glGenVertexArrays(1, &vertexArray);
    
    SceneFire* fire = static_cast<SceneFire*>(particles);
    updateParticleBuffers(fire);

    return true;
}


void FireProgram::updateParticleBuffers(SceneFire* fire)
{
    auto& ps = fire->particleSystem();
    numParticles = ps.numParticles();
    lifeTime = ps.lifeTime();
    emissionRange = ps.emissionRange();
    initialSpeedAverage = ps.initialSpeedAverage();
    initialSpeedVariation = ps.initialSpeedVariation();

    // Fill the first velocity buffer with random velocities
    Vector3f v;
    float speed, theta, phi;
    vector<GLfloat> data(numParticles * 3);
    srand(0);
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
    glBufferData(GL_ARRAY_BUFFER, data.size() * sizeof(GLfloat), &data.front(), GL_STATIC_DRAW);
    
    // Fill the offset time buffer
    data.resize(numParticles);
    float rate = lifeTime / numParticles;
    float time = 0.0f;
    for(GLuint i = 0; i < numParticles; ++i) {
        data[i] = time;
        time += rate;
    }
    glBindBuffer(GL_ARRAY_BUFFER, offsetTimeBuffer);
    glBufferData(GL_ARRAY_BUFFER, data.size() * sizeof(GLfloat), &data.front(), GL_STATIC_DRAW);
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


void FireProgram::render(SceneFire* fire)
{
    auto& ps = fire->particleSystem();

    if(!ps.on()){
        return;
    }

    if(numParticles != ps.numParticles() ||
       lifeTime != ps.lifeTime() ||
       emissionRange != ps.emissionRange() ||
       initialSpeedAverage != ps.initialSpeedAverage() ||
       initialSpeedVariation != ps.initialSpeedVariation()){
        updateParticleBuffers(fire);
    }
    
    setTime(fire->time() + ps.offsetTime());
    glUniform1f(lifeTimeLocation, ps.lifeTime());
    Vector3f accel = globalAttitude().transpose() * ps.acceleration();
    glUniform3fv(accelLocation, 1, accel.data());
    GLint blendSrc, blendDst;
    glGetIntegerv(GL_BLEND_SRC_ALPHA, &blendSrc);
    glGetIntegerv(GL_BLEND_DST_ALPHA, &blendDst);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    
    glBindVertexArray(vertexArray);
    glDrawArrays(GL_POINTS, 0, ps.numParticles());

    glBlendFunc(blendSrc, blendDst);
}
