/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "SceneParticles.h"
#include "ParticlesProgram.h"
#include <cnoid/EigenUtil>

using namespace std;
using namespace cnoid;

namespace {

struct NodeTypeRegistration {
    NodeTypeRegistration() {
        SgNode::registerType<SceneParticles, SgNode>();
    }
} registration;

}


SceneParticles::SceneParticles(int polymorhicId)
    : SgNode(polymorhicId)
{
    time_ = 0.0f;
}


SceneParticles::SceneParticles(const SceneParticles& org)
    : SgNode(org),
      time_(org.time_),
      texture_(org.texture_)
{

}
