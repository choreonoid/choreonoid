/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "SceneFountain.h"
#include <cnoid/GLSLSceneRenderer>
#include <cnoid/GLSLProgram>
#include <memory>

using namespace std;
using namespace cnoid;

namespace {

class FountainProgram : public GLSLProgram
{
public:
    FountainProgram(GLSLSceneRenderer* renderer);

    void render(SceneFountain* fountain);
};

}


void SceneFountain::initializeClass()
{
    SgNode::registerType<SceneFountain, SgNode>();

    GLSLSceneRenderer::addExtension(
        [](GLSLSceneRenderer* renderer){
            auto& functions = renderer->renderingFunctions();
            shared_ptr<FountainProgram> program = make_shared<FountainProgram>(renderer);
            functions.setFunction<SceneFountain>(
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


void FountainProgram::render(SceneFountain* fountain)
{

}

