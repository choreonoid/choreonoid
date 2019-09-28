/**
   @author Shin'ichiro Nakaoka
*/

#include "CoordinateAxesOverlay.h"
#include "SceneRenderer.h"
#include "MeshGenerator.h"
#include "EigenUtil.h"

using namespace cnoid;

namespace {

struct NodeTypeRegistration {
    NodeTypeRegistration() {
        SgNode::registerType<CoordinateAxesOverlay, SgOverlay>();

        SceneRenderer::addExtension(
            [](SceneRenderer* renderer){
                auto functions = renderer->renderingFunctions();
                functions->setFunction<CoordinateAxesOverlay>(
                    [=](SgNode* node){
                        static_cast<CoordinateAxesOverlay*>(node)->render(renderer);
                    });
            });
    }
} registration;

}


CoordinateAxesOverlay::CoordinateAxesOverlay()
    : SgOverlay(findPolymorphicId<CoordinateAxesOverlay>())
{
    static const Vector3f colors[] = {
        { 1.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f },
        { 0.4f, 0.6f, 1.0f }};
        
    MeshGenerator meshGenerator;
    meshGenerator.setBoundingBoxUpdateEnabled(false);
    SgMeshPtr mesh = meshGenerator.generateArrow(width / 2.0, length, width, width * 2.0);
    mesh->translate(Vector3f(0.0f, length / 2.0, 0.0f));
    mesh->updateBoundingBox();
    
    //auto invariant = new SgInvariantGroup;
    
    axesTransform = new SgPosTransform;
    
    for(int i=0; i < 3; ++i){
        SgShape* shape = new SgShape;
        shape->setMesh(mesh);
        SgMaterial* material = new SgMaterial;
        material->setDiffuseColor(colors[i]);
        shape->setMaterial(material);
        SgPosTransform* transform = new SgPosTransform;
        transform->addChild(shape);
        if(i == 0){
            transform->setRotation(AngleAxis(-PI / 2.0, Vector3::UnitZ()));
        } else if(i == 2){
            transform->setRotation(AngleAxis( PI / 2.0, Vector3::UnitX()));
        }
        axesTransform->addChild(transform);
    }
    addChild(axesTransform);
}


void CoordinateAxesOverlay::calcViewVolume(double viewportWidth, double viewportHeight, ViewVolume& io_volume)
{
    const double margin = 1.4 * (length + width * 2.0);
    io_volume.left = -margin;
    io_volume.right = viewportWidth - margin;
    io_volume.top = viewportHeight - margin;
    io_volume.bottom = -margin;
    io_volume.zNear = -margin;
    io_volume.zFar = margin;
}


void CoordinateAxesOverlay::render(SceneRenderer* renderer)
{
    const Affine3& T = renderer->currentCameraPosition();
    axesTransform->setRotation(T.linear().transpose());
    renderer->renderingFunctions()->dispatchAs<SgOverlay>(this);
}
