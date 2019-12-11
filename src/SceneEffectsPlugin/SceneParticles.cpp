/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "SceneParticles.h"
#include "ParticlesProgram.h"
#include <cnoid/SceneNodeClassRegistry>
#include <cnoid/EigenUtil>

using namespace std;
using namespace cnoid;

namespace {

struct NodeClassRegistration {
    NodeClassRegistration() {
        SceneNodeClassRegistry::instance().registerClass<SceneParticles, SgNode>();
    }
} registration;

}


SceneParticles::SceneParticles(int classId)
    : SgNode(classId)
{
    time_ = 0.0f;
}


SceneParticles::SceneParticles(const SceneParticles& org)
    : SgNode(org),
      time_(org.time_),
      texture_(org.texture_)
{

}
