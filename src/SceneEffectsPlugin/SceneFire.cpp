/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "SceneEffectsPlugin.h"
#include "SceneFire.h"
#include "ParticlesProgram.h"
#include <cnoid/EigenUtil>

using namespace std;
using namespace cnoid;

namespace {

class FireProgram : public LuminousParticlesProgram
{
public:
    FireProgram(GLSLSceneRenderer* renderer);
    virtual bool initializeRendering(SceneParticles* particles) override;
    void render(SceneFire* fountain);

    GLint lifeTimeLocation;
    GLint accelLocation;
    GLuint nParticles;
    GLuint initVelBuffer;
    GLuint offsetTimeBuffer;
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
    lifeTime_ = 2.5f;
    acceleration_ << 0.0f, 0.0f, 0.1f;
    initialSpeedAverage_ = 0.15f;
    initialSpeedVariation_ = 0.1f;
    initialVelocityAngleRange_ = PI / 3.0f;
    setParticleSize(0.15f);
    setTexture(":/SceneEffectsPlugin/texture/fire.png");
}


SceneFire::SceneFire(const SceneFire& org)
    : SceneParticles(org)
{
    lifeTime_ = org.lifeTime_;
    acceleration_ = org.acceleration_;
    initialSpeedAverage_ = org.initialSpeedAverage_;
    initialSpeedVariation_ = org.initialSpeedVariation_;
    initialVelocityAngleRange_ = org.initialVelocityAngleRange_;
}


SgObject* SceneFire::clone(SgCloneMap& cloneMap) const
{
    return new SceneFire(*this);
}


FireProgram::FireProgram(GLSLSceneRenderer* renderer)
    : LuminousParticlesProgram(renderer)
{

}


bool FireProgram::initializeRendering(SceneParticles* particles)
{
    loadVertexShader(":/SceneEffectsPlugin/shader/Fire.vert");
    loadFragmentShader(":/SceneEffectsPlugin/shader/LuminousParticles.frag");
    link();
    
    if(!ParticlesProgramBase::initializeRendering(particles)){
        return false;
    }
    SceneFire* fire = static_cast<SceneFire*>(particles);

    nParticles = 500;

    // Fill the first velocity buffer with random velocities
    Vector3f v;
    float speed, theta, phi;
    vector<GLfloat> data(nParticles * 3);
    for(GLuint i = 0; i < nParticles; ++i) {
        theta = fire->initialVelocityAngleRange() * random();
        phi = 2.0 * PI * random();

        v.x() = sinf(theta) * cosf(phi);
        v.y() = sinf(theta) * sinf(phi);
        v.z() = cosf(theta);

        speed = std::max(0.0f, fire->initialSpeedAverage() + fire->initialSpeedVariation() * random());
        v = v.normalized() * speed;

        data[3*i]   = v.x();
        data[3*i+1] = v.y();
        data[3*i+2] = v.z();
    }
    glGenBuffers(1, &initVelBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, initVelBuffer);
    glBufferData(GL_ARRAY_BUFFER, data.size() * sizeof(GLfloat), &data.front(), GL_STATIC_DRAW);
    
    // Fill the offset time buffer
    data.resize(nParticles);
    float rate = fire->lifeTime() / nParticles;
    float time = 0.0f;
    for(GLuint i = 0; i < nParticles; ++i) {
        data[i] = time;
        time += rate;
    }
    glGenBuffers(1, &offsetTimeBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, offsetTimeBuffer);
    glBufferData(GL_ARRAY_BUFFER, data.size() * sizeof(GLfloat), &data.front(), GL_STATIC_DRAW);
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


void FireProgram::render(SceneFire* fire)
{
    setTime(fire->time());
    glUniform1f(lifeTimeLocation, fire->lifeTime());
    Vector3f accel = globalAttitude().transpose() * fire->acceleration();
    glUniform3fv(accelLocation, 1, accel.data());
    GLint blendSrc, blendDst;
    glGetIntegerv(GL_BLEND_SRC_ALPHA, &blendSrc);
    glGetIntegerv(GL_BLEND_DST_ALPHA, &blendDst);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    
    glBindVertexArray(vertexArray);
    glDrawArrays(GL_POINTS, 0, nParticles);

    glBlendFunc(blendSrc, blendDst);
}
