#include "CoordinateAxesOverlay.h"
#include "SceneDrawables.h"  // for registerSceneDrawableNodeClasses()
#include "SceneNodeClassRegistry.h"
#include "SceneRenderer.h"
#include "MeshGenerator.h"
#include "EigenUtil.h"

using namespace cnoid;

namespace {

struct NodeClassRegistration {
    NodeClassRegistration() {
        // Ensure parent class SgViewportOverlay is registered first
        registerSceneDrawableNodeClasses();
        
        SceneNodeClassRegistry::instance().
            registerClass<CoordinateAxesOverlay, SgViewportOverlay>();

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


CoordinateAxesOverlay::CoordinateAxesOverlay(CoordinateSystem coordinateSystem)
    : SgViewportOverlay(findClassId<CoordinateAxesOverlay>()),
      superClassId(findClassId<SgViewportOverlay>())
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
    
    topTransform = new SgPosTransform;
    
    for(int i=0; i < 3; ++i){
        SgShape* shape = new SgShape;
        shape->setMesh(mesh);
        SgMaterial* material = new SgMaterial;
        material->setDiffuseColor(colors[i]);
        shape->setMaterial(material);
        auto axisTransform = new SgPosTransform;
        axisTransform->addChild(shape);
        topTransform->addChild(axisTransform);
        axisTransforms[i] = axisTransform;
    }
    axisTransforms[0]->setRotation(AngleAxis(-PI / 2.0, Vector3::UnitZ()));
    axisTransforms[2]->setRotation(AngleAxis( PI / 2.0, Vector3::UnitX()));

    setCoordinateSystem(coordinateSystem);

    addChild(topTransform);
}


void CoordinateAxesOverlay::setCoordinateSystem(CoordinateSystem coordinateSystem)
{
    // Set the direction of Y-axis
    if(coordinateSystem == RightHanded){
        axisTransforms[1]->setRotation(Matrix3::Identity());
    } else {
        axisTransforms[1]->setRotation(AngleAxis(PI, Vector3::UnitX()));
    }
    axisTransforms[1]->notifyUpdate();
}


void CoordinateAxesOverlay::calcViewVolume(double viewportWidth, double viewportHeight, ViewVolume& io_volume)
{
    const double margin = 1.4 * (length + width * 2.0);
    io_volume.left = -margin;
    io_volume.right = viewportWidth - margin;
    io_volume.top = viewportHeight - margin;
    io_volume.bottom = -margin;
    io_volume.zNear = margin;
    io_volume.zFar = -margin;
}


void CoordinateAxesOverlay::render(SceneRenderer* renderer)
{
    const Isometry3& T = renderer->currentCameraPosition();
    topTransform->setRotation(T.linear().transpose());
    renderer->renderingFunctions()->dispatch(this, superClassId);
}
