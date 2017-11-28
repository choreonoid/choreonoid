/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_SCENE_EFFECTS_PLUGIN_H
#define CNOID_SCENE_EFFECTS_PLUGIN_H

#include <cnoid/GLSLSceneRenderer>
#include <memory>

namespace cnoid {

template<class SceneNode, class Program>
static void registerSceneEffectType(){
    GLSLSceneRenderer::addExtension(
        [](GLSLSceneRenderer* renderer){
            auto program = std::make_shared<Program>(renderer);
            renderer->renderingFunctions()->setFunction<SceneNode>(
                [program](SceneNode* particles){
                    program->requestRendering(particles, [program, particles]() { program->render(particles); });
                });
        });
}

}

#endif
