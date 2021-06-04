/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "SceneCollision.h"
#include <cnoid/SceneNodeClassRegistry>
#include <cnoid/SceneRenderer>

using namespace std;
using namespace cnoid;

namespace {

struct NodeClassRegistration {
    NodeClassRegistration() {
        SceneNodeClassRegistry::instance().registerClass<SceneCollision, SgLineSet>();

        SceneRenderer::addExtension(
            [](SceneRenderer* renderer){
                auto functions = renderer->renderingFunctions();
                functions->setFunction<SceneCollision>(
                    [renderer](SgNode* node){
                        static_cast<SceneCollision*>(node)->render(renderer);
                    });
            });
    }
} registration;

}


SceneCollision::SceneCollision(std::shared_ptr<std::vector<CollisionLinkPairPtr>> collisionPairs)
    : SgLineSet(findClassId<SceneCollision>()),
      collisionPairs(collisionPairs)
{
    vertices_ = setVertices(new SgVertexArray);
    setMaterial(new SgMaterial)->setDiffuseColor(Vector3f(0.0f, 1.0f, 0.0f));
    isDirty = true;
}


SceneCollision::SceneCollision(const SceneCollision&)
{

}


void SceneCollision::render(SceneRenderer* renderer)
{
    static const SceneRenderer::PropertyKey key("collisionLineRatio");

    const double collisionLineRatio = renderer->property(key, 0.0);
    if(collisionLineRatio <= 0.0){
        return;
    }
    
    if(isDirty){
        vertices_->clear();
        lineVertexIndices().clear();
        for(size_t i=0; i < collisionPairs->size(); ++i){
            const CollisionLinkPair& pair = *(*collisionPairs)[i];
            const vector<Collision>& cols = pair.collisions;

            // flip the line direction so that the line is always from the staic object to the dynamic one
            double direction = 1.0;
            if(pair.body[1] && pair.body[0]){
                direction = (pair.body[1]->isStaticModel() && !pair.body[0]->isStaticModel()) ? -1.0 : 1.0;
            }

            for(size_t j=0; j < cols.size(); ++j){
                const Collision& c = cols[j];
                const int index = vertices_->size();
                addLine(index, index + 1);
                vertices_->push_back(c.point.cast<float>());
                vertices_->push_back((c.point + direction * collisionLineRatio * c.depth * c.normal).cast<float>());
            }
        }
        vertices_->notifyUpdate();
        isDirty = false;
    }
    
    renderer->renderingFunctions()->dispatchAs<SgLineSet>(this);
}
