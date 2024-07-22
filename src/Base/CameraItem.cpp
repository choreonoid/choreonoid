#include "CameraItem.h"
#include "CameraConfigDialog.h"
#include <cnoid/ItemManager>
#include <cnoid/MenuManager>
#include <cnoid/InteractiveCameraTransform>
#include <cnoid/ItemTreeView>
#include <cnoid/SceneView>
#include <cnoid/SceneWidget>
#include <cnoid/SceneRenderer>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include <cnoid/SceneCameras>
#include <cnoid/EigenUtil>
#include <cnoid/EigenArchive>
#include "gettext.h"

using namespace std;
using namespace cnoid;


namespace cnoid {

class CameraItem::Impl
{
public:
    CameraItem* self;
    InteractiveCameraTransformPtr cameraTransform;
    SgPerspectiveCameraPtr persCamera;
    SgOrthographicCameraPtr orthoCamera;
    SgCameraPtr currentCamera;
    SgUpdate update;
    Selection cameraType;
    
    Impl(CameraItem* self, InteractiveCameraTransform* cameraTransform, bool initCameraPosition);
    Impl(CameraItem* self, const Impl& org, InteractiveCameraTransform* newCameraTransform);
    void doPutProperties(PutPropertyFunction& putProperty);
    bool setClipDistances(double nearDistance, double farDistance);
    bool setFieldOfView(double fov);
    bool setCameraType(int index);
};

}


void CameraItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager()
        .registerClass<CameraItem>(N_("CameraItem"))
        .addCreationPanel<CameraItem>();

    ItemTreeView::customizeContextMenu<CameraItem>(
        [](CameraItem* item, MenuManager& menuManager, ItemFunctionDispatcher menuFunction){
            menuManager.addItem(_("Activate camera"))->sigTriggered().connect(
                [item](){ item->activateCameraInSceneView(SceneView::lastFocusSceneView()); });
            menuManager.addItem(_("Apply to built-in camera"))->sigTriggered().connect(
                [item](){
                    item->activateSceneViewBuiltinCameraWithCameraItemConfiguration(
                        SceneView::lastFocusSceneView());
                });
            menuManager.addItem(_("Camera configuration"))->sigTriggered().connect(
                [item](){ item->showDialogToConfigureCamera(); });
            menuManager.addSeparator();
            menuFunction.dispatchAs<Item>(item);
        });
}


CameraItem* CameraItem::showDialogToCreateCameraItem(Item* parentItem)
{
    return CameraConfigDialog::instance()->showToCreateCameraItem(parentItem);
}


void CameraItem::showDialogToConfigureCamera()
{
    CameraConfigDialog::instance()->showToConfigureCameraItem(this);
}


CameraItem::CameraItem()
    : Item("Camera")
{
    impl = new Impl(this, new InteractiveCameraTransform, true);
}


CameraItem::CameraItem(const char* name, InteractiveCameraTransform* cameraTransform)
    : Item(name)
{
    impl = new Impl(this, cameraTransform, true);
}


CameraItem::Impl::Impl(CameraItem* self, InteractiveCameraTransform* cameraTransform, bool initCameraPosition)
    : self(self),
      cameraType(NumCameraTypes, CNOID_GETTEXT_DOMAIN_NAME),
      cameraTransform(cameraTransform)
{
    cameraType.setSymbol(Perspective,  N_("Perspective"));
    cameraType.setSymbol(Orthographic, N_("Orthographic"));
    cameraType.select(Perspective);

    if(initCameraPosition){
        cameraTransform->setPosition(
            SceneView::instance()->sceneWidget()->builtinCameraTransform()->position());
    } else {
        cameraTransform->setInteractiveViewpointChangeLocked(true);
    }

    persCamera = new SgPerspectiveCamera;
    persCamera->setName(self->name());
    cameraTransform->addChild(persCamera);
    currentCamera = persCamera;

    orthoCamera = new SgOrthographicCamera;
    orthoCamera->setName(self->name());
}


CameraItem::CameraItem(const CameraItem& org)
    : Item(org)
{
    impl = new Impl(this, *org.impl, new InteractiveCameraTransform);
}


CameraItem::CameraItem(const CameraItem& org, InteractiveCameraTransform* newCameraTransform)
    : Item(org)
{
    impl = new Impl(this, *org.impl, newCameraTransform);
}


CameraItem::Impl::Impl(CameraItem* self, const Impl& org, InteractiveCameraTransform* newCameraTransform)
    : Impl(self, newCameraTransform, false)
{
    cameraType = org.cameraType;

    cameraTransform->setPosition(org.cameraTransform->position());
    cameraTransform->setInteractiveViewpointChangeLocked(
        org.cameraTransform->isInteractiveViewpointChangeLocked());
}


CameraItem::~CameraItem()
{
    delete impl;
}


bool CameraItem::setName(const std::string& name_)
{
    if(name_ != name()){
        impl->persCamera->setName(name_);
        impl->orthoCamera->setName(name_);
        Item::setName(name_);
        impl->persCamera->notifyUpdate(impl->update);
        impl->orthoCamera->notifyUpdate(impl->update);
    }
    return true;
}


Item* CameraItem::doCloneItem(CloneMap* /* cloneMap */) const
{
    return new CameraItem(*this);
}


void CameraItem::setInteractiveViewpointChangeEnabled(bool on)
{
    impl->cameraTransform->setInteractiveViewpointChangeLocked(!on);
}


bool CameraItem::isInteractiveViewpointChangeEnabled() const
{
    return !impl->cameraTransform->isInteractiveViewpointChangeLocked();
}


void CameraItem::setCameraType(CameraType type)
{
    impl->setCameraType(type);
}


/**
   \todo Improve the scene widget so that the current camera path described in a string list
   can be kept even if the actual camera node is changed, and simplify the following implementation.
*/
bool CameraItem::Impl::setCameraType(int index)
{
    if(cameraType.selectedIndex() == index){
        return true;
    }

    cameraType.select(index);

    SgCamera* cameraToRemove;
    if(cameraType.is(CameraItem::Perspective)){
        currentCamera = persCamera;
        cameraToRemove = orthoCamera;
    }else if(cameraType.is(CameraItem::Orthographic)){
        currentCamera = orthoCamera;
        cameraToRemove = persCamera;
    }

    vector<SceneRenderer*> renderers;
    for(auto sceneView : SceneView::instances()){
        renderers.push_back(sceneView->sceneWidget()->renderer());
    }

    cameraTransform->addChild(currentCamera, update);
    for(auto renderer : renderers){
        if(renderer->currentCamera() == cameraToRemove){
            renderer->extractPreprocessedNodes();
            renderer->setCurrentCamera(currentCamera);
        }
    }
    cameraTransform->removeChild(cameraToRemove, update);
    for(auto renderer : renderers){
        renderer->extractPreprocessedNodes();
    }

    return true;
}


CameraItem::CameraType CameraItem::cameraType() const
{
    return static_cast<CameraType>(impl->cameraType.which());
}


SgPerspectiveCamera* CameraItem::perspectiveCamera()
{
    return impl->persCamera;
}


SgOrthographicCamera* CameraItem::orthographicCamera()
{
    return impl->orthoCamera;
}


SgCamera* CameraItem::currentCamera()
{
    return impl->currentCamera;
}


SgPosTransform* CameraItem::cameraTransform()
{
    return impl->cameraTransform;
}


SgNode* CameraItem::getScene()
{
    return impl->cameraTransform;
}


double CameraItem::fieldOfView() const
{
    return impl->persCamera->fieldOfView();
}


bool CameraItem::setFieldOfView(double fov)
{
    return impl->setFieldOfView(fov);
}


bool CameraItem::Impl::setFieldOfView(double fov)
{
    if(fov > 0.0 && fov < PI){
        persCamera->setFieldOfView(fov);
        persCamera->notifyUpdate(update);
        return true;
    }
    return false;
}


double CameraItem::nearClipDistance() const
{
    return impl->persCamera->nearClipDistance();
}


bool CameraItem::setNearClipDistance(double nearDistance)
{
    if(nearDistance > 0.0){
        impl->setClipDistances(nearDistance, farClipDistance());
        return true;
    }
    return false;
}


double CameraItem::farClipDistance() const
{
    return impl->persCamera->farClipDistance();
}


bool CameraItem::setFarClipDistance(double farDistance)
{
    if(farDistance > 0.0){
        impl->setClipDistances(nearClipDistance(), farDistance);
        return true;
    }
    return false;
}


bool CameraItem::setClipDistances(double nearDistance, double farDistance)
{
    if(nearDistance > 0.0 && farDistance > 0.0){
        impl->setClipDistances(nearDistance, farDistance);
        return true;
    }
    return false;
}


bool CameraItem::Impl::setClipDistances(double nearDistance, double farDistance)
{
    if(persCamera->nearClipDistance() != nearDistance || persCamera->farClipDistance() != farDistance){
        persCamera->setNearClipDistance(nearDistance);
        persCamera->setFarClipDistance(farDistance);
        orthoCamera->setNearClipDistance(nearDistance);
        orthoCamera->setFarClipDistance(farDistance);
        persCamera->notifyUpdate(update);
        orthoCamera->notifyUpdate(update);
    }
    return true;
}


void CameraItem::activateCameraInSceneView(SceneView* sceneView, bool on)
{
    auto sceneWidget = sceneView->sceneWidget();
    auto renderer = sceneWidget->renderer();

    if(on && !isChecked()){
        setChecked(true);
        renderer->extractPreprocessedNodes();
    }

    auto sceneViewCamera = renderer->currentCamera();
    if(on){
        if(impl->currentCamera != sceneViewCamera){
            renderer->setCurrentCamera(impl->currentCamera);
        }
    } else {
        if(impl->currentCamera == sceneViewCamera){
            if(impl->currentCamera == impl->persCamera){
                renderer->setCurrentCamera(sceneWidget->builtinPerspectiveCamera());
            } else {
                renderer->setCurrentCamera(sceneWidget->builtinOrthographicCamera());
            }
        }
    }
}


void CameraItem::activateSceneViewBuiltinCameraWithCameraItemConfiguration(SceneView* sceneView)
{
    auto sceneWidget = SceneView::instance()->sceneWidget();
    auto builtinCameraTransform = sceneWidget->builtinCameraTransform();
    builtinCameraTransform->setPosition(impl->cameraTransform->position());

    SgCamera* builtinCamera = nullptr;
    if(impl->currentCamera == impl->persCamera){
        auto builtinPersCamera = sceneWidget->builtinPerspectiveCamera();
        builtinPersCamera->setFieldOfView(impl->persCamera->fieldOfView());
        builtinCamera = builtinPersCamera;
    } else if(impl->currentCamera == impl->orthoCamera){
        auto builtinOrthoCamera = sceneWidget->builtinOrthographicCamera();
        builtinOrthoCamera->setHeight(impl->orthoCamera->height());
        builtinCamera = builtinOrthoCamera;
    }

    if(builtinCamera){
        builtinCamera->setNearClipDistance(impl->currentCamera->nearClipDistance());
        builtinCamera->setFarClipDistance(impl->currentCamera->farClipDistance());
        sceneWidget->renderer()->setCurrentCamera(builtinCamera);
        builtinCamera->notifyUpdate();
    }
}


void CameraItem::onDoubleClicked()
{
    activateSceneViewBuiltinCameraWithCameraItemConfiguration(SceneView::lastFocusSceneView());
}


void CameraItem::doPutProperties(PutPropertyFunction& putProperty)
{
    impl->doPutProperties(putProperty);
}


void CameraItem::Impl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Camera type"), cameraType,
                [&](int index){ return setCameraType(index); });
    putProperty(_("Field Of View"), degree(self->fieldOfView()),
                [&](double fov){ return setFieldOfView(radian(fov)); } );
    putProperty(_("Near clip distance"), self->nearClipDistance(),
                [&](double distance){ return self->setNearClipDistance(distance); });
    putProperty(_("Far clip distance"), self->farClipDistance(),
                [&](double distance){ return self->setFarClipDistance(distance); } );
    putProperty(_("Interactive viewpoint change"), self->isInteractiveViewpointChangeEnabled(),
                [&](bool on){ self->setInteractiveViewpointChangeEnabled(on); return true; });
}


bool CameraItem::store(Archive& archive)
{
    archive.write("camera_type", impl->cameraType.selectedSymbol());

    auto transform = impl->cameraTransform;
    Isometry3 T = transform->T();
    write(archive, "translation", T.translation());
    writeDegreeAngleAxis(archive, "rotation", AngleAxis(T.linear()));
    
    archive.write("field_of_view", impl->persCamera->fieldOfView());
    archive.write("near_clip_distance", impl->persCamera->nearClipDistance());    
    archive.write("far_clip_distance", impl->persCamera->farClipDistance());
    archive.write("interactive_viewpoint_change", isInteractiveViewpointChangeEnabled());

    return true;
}


bool CameraItem::restore(const Archive& archive)
{
    string symbol;
    if(archive.read({ "camera_type", "cameraType" }, symbol)){
        int index = impl->cameraType.index(symbol);
        impl->setCameraType(index);
    }
    
    auto transform = impl->cameraTransform;
    Isometry3 T = Isometry3::Identity();
    Vector3 p;
    if(read(archive, "translation", p)){
        T.translation() = p;
    }
    AngleAxis aa;
    if(readDegreeAngleAxis(archive, "rotation", aa)){
        T.linear() = aa.toRotationMatrix();
    }
    transform->setPosition(T);

    impl->setFieldOfView(archive.get({ "field_of_view", "fieldOfView" }, impl->persCamera->fieldOfView()));

    double nearDistance =
        archive.get({ "near_clip_distance", "nearClipDistance" }, impl->persCamera->nearClipDistance());
    double farDistance =
        archive.get({ "far_clip_distance", "farClipDistance" }, impl->persCamera->farClipDistance());
    impl->setClipDistances(nearDistance, farDistance);

    bool on;
    if(archive.read("interactive_viewpoint_change", on)){
        setInteractiveViewpointChangeEnabled(on);
    }

    transform->notifyUpdate();

    return true;
}
