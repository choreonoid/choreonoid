/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "SceneFountain.h"
#include <cnoid/GLSLSceneRenderer>
#include <cnoid/ShaderPrograms>
#include <memory>

using namespace std;
using namespace cnoid;

namespace {

class FountainProgram : public NolightingProgram
{
public:
    FountainProgram(GLSLSceneRenderer* renderer);

    virtual void initialize() override;
    virtual void activate() override;
    virtual void initializeRendering() override;
    virtual void deactivate() override;

    void render(SceneFountain* fountain);

    GLint timeLocation;
    GLint lifeTimeLocation;
};

}


void SceneFountain::initializeClass()
{
    SgNode::registerType<SceneFountain, SgNode>();

    GLSLSceneRenderer::addExtension(
        [](GLSLSceneRenderer* renderer){
            shared_ptr<FountainProgram> program = make_shared<FountainProgram>(renderer);
            renderer->renderingFunctions().setFunction<SceneFountain>(
                [program](SceneFountain* fountain){
                    program->render(fountain);
                });
        });
}


SceneFountain::SceneFountain()
    : SgNode(findPolymorphicId<SceneFountain>())
{

}


FountainProgram::FountainProgram(GLSLSceneRenderer* renderer)
{

}


void FountainProgram::initialize()
{
    loadVertexShader(":/PhenomenonPlugin/shader/fountain.vert");
    loadFragmentShader(":/PhenomenonPlugin/shader/fountain.frag");
    link();
    
    NolightingProgram::initialize();
    
    timeLocation = getUniformLocation("time");
    lifeTimeLocation = getUniformLocation("lifeTime");
}


void FountainProgram::activate()
{
    NolightingProgram::activate();
}
    

void FountainProgram::initializeRendering()
{

}


void FountainProgram::deactivate()
{

}


void FountainProgram::render(SceneFountain* fountain)
{

}
