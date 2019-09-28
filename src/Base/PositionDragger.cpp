/**
   @author Shin'ichiro Nakaoka
*/

#include "PositionDragger.h"
#include "TranslationDragger.h"
#include "RotationDragger.h"
#include "SceneDragProjector.h"
#include <cnoid/SceneWidget>
#include <deque>

using namespace std;
using namespace cnoid;

namespace cnoid {

class PositionDraggerImpl
{
public:
    PositionDragger* self;
    TranslationDraggerPtr translationDragger;
    RotationDraggerPtr rotationDragger;
    SceneDragProjector dragProjector;
    int draggableAxes;
    bool isContentsDragEnabled;
    bool isDraggerAlwaysShown;
    bool isDraggerAlwaysHidden;
    bool isDraggerShown;
    bool isUndoEnabled;
    std::deque<Affine3> history;
    Signal<void(int axisSet)> sigDraggableAxesChanged;
    Signal<void()> sigDragStarted;
    Signal<void()> sigPositionDragged;
    Signal<void()> sigDragFinished;

    PositionDraggerImpl(PositionDragger* self);
    PositionDraggerImpl(PositionDragger* self, const PositionDraggerImpl& org);
    PositionDraggerImpl(PositionDragger* self, const PositionDraggerImpl& org, SgCloneMap* cloneMap);
    void initializeDraggers();
    void onSubDraggerDragStarted();
    void onSubDraggerDragged();
    void storeCurrentPositionToHistory();
    void showDragMarkers(bool on);
};

}


PositionDragger::PositionDragger()
{
    impl = new PositionDraggerImpl(this);
}


PositionDraggerImpl::PositionDraggerImpl(PositionDragger* self)
    : self(self)
{
    translationDragger = new TranslationDragger;
    rotationDragger = new RotationDragger;
    draggableAxes =
        PositionDragger::TX | PositionDragger::TY | PositionDragger::TZ |
        PositionDragger::RX | PositionDragger::RY | PositionDragger::RZ;

    initializeDraggers();
    
    isDraggerAlwaysShown = false;
    isDraggerAlwaysHidden = false;
    isContentsDragEnabled = true;
    isUndoEnabled = false;    
}


PositionDragger::PositionDragger(const PositionDragger& org)
    : SceneDragger(org)
{
    impl = new PositionDraggerImpl(this, *org.impl);
}


PositionDraggerImpl::PositionDraggerImpl(PositionDragger* self, const PositionDraggerImpl& org)
    : self(self)
{
    translationDragger = new TranslationDragger(*org.translationDragger);
    rotationDragger = new RotationDragger(*org.rotationDragger);
    draggableAxes = org.draggableAxes;

    initializeDraggers();

    isDraggerAlwaysShown = org.isDraggerAlwaysShown;
    isDraggerAlwaysHidden = org.isDraggerAlwaysHidden;
    isContentsDragEnabled = org.isContentsDragEnabled;
    isUndoEnabled = org.isUndoEnabled;
}


PositionDragger::PositionDragger(const PositionDragger& org, SgCloneMap* cloneMap)
    : SceneDragger(org, cloneMap)
{
    impl = new PositionDraggerImpl(this, *org.impl, cloneMap);
}


PositionDraggerImpl::PositionDraggerImpl(PositionDragger* self, const PositionDraggerImpl& org, SgCloneMap* cloneMap)
    : self(self)
{
    translationDragger = new TranslationDragger(*org.translationDragger, cloneMap);
    rotationDragger = new RotationDragger(*org.rotationDragger, cloneMap);
    draggableAxes = org.draggableAxes;

    initializeDraggers();

    isDraggerAlwaysShown = org.isDraggerAlwaysShown;
    isDraggerAlwaysHidden = org.isDraggerAlwaysHidden;
    isContentsDragEnabled = org.isContentsDragEnabled;
    isUndoEnabled = org.isUndoEnabled;
}


void PositionDraggerImpl::initializeDraggers()
{
    translationDragger->sigTranslationStarted().connect(
        std::bind(&PositionDraggerImpl::onSubDraggerDragStarted, this));
    translationDragger->sigTranslationDragged().connect(
        std::bind(&PositionDraggerImpl::onSubDraggerDragged, this));
    translationDragger->sigTranslationFinished().connect(std::ref(sigDragFinished));
    
    rotationDragger->sigRotationStarted().connect(
        std::bind(&PositionDraggerImpl::onSubDraggerDragStarted, this));
    rotationDragger->sigRotationDragged().connect(
        std::bind(&PositionDraggerImpl::onSubDraggerDragged, this));
    rotationDragger->sigRotationFinished().connect(std::ref(sigDragFinished));
}


void PositionDragger::setDraggableAxes(int axisSet)
{
    if(axisSet != impl->draggableAxes){
        int translationAxes = axisSet & (TX | TY | TZ);
        impl->translationDragger->setDraggableAxes(translationAxes);
        int rotationAxes = (axisSet & (RX | RY | RZ)) >> 3;
        impl->rotationDragger->setDraggableAxes(rotationAxes);
        impl->draggableAxes = axisSet;
        impl->sigDraggableAxesChanged(axisSet);
    }
}


int PositionDragger::draggableAxes() const
{
    return impl->draggableAxes;
}


SignalProxy<void(int axisSet)> PositionDragger::sigDraggableAxesChanged()
{
    return impl->sigDraggableAxesChanged;
}


SgObject* PositionDragger::doClone(SgCloneMap* cloneMap) const
{
    return new PositionDragger(*this, cloneMap);
}


TranslationDragger* PositionDragger::translationDragger()
{
    return impl->translationDragger;
}


RotationDragger* PositionDragger::rotationDragger()
{
    return impl->rotationDragger;
}


SignalProxy<void()> PositionDragger::sigDragStarted()
{
    return impl->sigDragStarted;
}


SignalProxy<void()> PositionDragger::sigPositionDragged()
{
    return impl->sigPositionDragged;
}


SignalProxy<void()> PositionDragger::sigDragFinished()
{
    return impl->sigDragFinished;
}


void PositionDragger::setRadius(double r, double translationAxisRatio)
{
    impl->translationDragger->setRadius(r * translationAxisRatio);
    impl->rotationDragger->setRadius(r);
}


void PositionDragger::adjustSize(const BoundingBox& bb)
{
    if(!bb.empty()){
        Vector3 s = bb.size() / 2.0;
        std::sort(s.data(), s.data() + 3);
        double a = Vector2(s[0], s[1]).norm() * 1.1;
        double r = std::max(a, s[2] * 1.2);
        setRadius(r);
    }
}


void PositionDragger::adjustSize()
{
    BoundingBox bb;
    for(int i=0; i < numChildren(); ++i){
        SgNode* node = child(i);
        if(node != impl->translationDragger && node != impl->rotationDragger){
            bb.expandBy(node->boundingBox());
        }
    }
    adjustSize(bb);
}


void PositionDragger::setContentsDragEnabled(bool on)
{
    impl->isContentsDragEnabled = on;
}


bool PositionDragger::isContentsDragEnabled() const
{
    return impl->isContentsDragEnabled;
}


void PositionDragger::setDraggerAlwaysShown(bool on)
{
    if(on){
        impl->isDraggerAlwaysHidden = false;
    }
    bool changed = (on != impl->isDraggerAlwaysShown);
    impl->isDraggerAlwaysShown = on;
    if(on && changed){
        impl->showDragMarkers(true);
    }
}


bool PositionDragger::isDraggerAlwaysShown() const
{
    return impl->isDraggerAlwaysShown;
}


void PositionDragger::setDraggerAlwaysHidden(bool on)
{
    if(on){
        impl->isDraggerAlwaysShown = false;
    }
    bool changed = (on != impl->isDraggerAlwaysHidden);
    impl->isDraggerAlwaysHidden = on;
    if(on && changed){
        impl->showDragMarkers(false);
    }
}


bool PositionDragger::isDraggerAlwaysHidden() const
{
    return impl->isDraggerAlwaysHidden;
}


void PositionDraggerImpl::showDragMarkers(bool on)
{
    if(isDraggerAlwaysHidden){
        on = false;
    } else if(isDraggerAlwaysShown){
        on = true;
    }
    
    if(on){
        self->addChildOnce(translationDragger, true);
        self->addChildOnce(rotationDragger, true);
    } else {
        self->removeChild(translationDragger, true);
        self->removeChild(rotationDragger, true);
    }
}    


bool PositionDragger::isDragging() const
{
    return (impl->translationDragger->isDragging() ||
            impl->rotationDragger->isDragging() ||
            impl->dragProjector.isDragging());
}


Affine3 PositionDragger::draggedPosition() const
{
    if(impl->rotationDragger->isDragging()){
        return impl->rotationDragger->draggedPosition();
    } else if(impl->translationDragger->isDragging()){
        return impl->translationDragger->draggedPosition();
    } else if(impl->dragProjector.isDragging()){
        return impl->dragProjector.position();
    } else {
        return T();
    }
}


void PositionDraggerImpl::onSubDraggerDragStarted()
{
    storeCurrentPositionToHistory();
    sigDragStarted();
}


void PositionDraggerImpl::onSubDraggerDragged()
{
    if(self->isContainerMode()){
        if(isContentsDragEnabled){
            self->setPosition(self->draggedPosition());
            self->notifyUpdate();
            sigPositionDragged();
        }
    } else {
        sigPositionDragged();
    }
}


bool PositionDragger::onButtonPressEvent(const SceneWidgetEvent& event)
{
    if(isContainerMode() && impl->isContentsDragEnabled){
        if(!impl->isDraggerAlwaysShown){
            impl->showDragMarkers(true);
        }
        impl->dragProjector.setInitialPosition(T());
        impl->dragProjector.setTranslationAlongViewPlane();
        if(impl->dragProjector.startTranslation(event)){
            storeCurrentPositionToHistory();
            impl->sigDragStarted();
            return true;
        }
    }
    return false;
}


bool PositionDragger::onButtonReleaseEvent(const SceneWidgetEvent&)
{
    if(isContainerMode() && impl->isContentsDragEnabled){
        if(impl->dragProjector.isDragging()){
            impl->sigDragFinished();
            impl->dragProjector.resetDragMode();
            return true;
        }
    }
    return false;
}


bool PositionDragger::onPointerMoveEvent(const SceneWidgetEvent& event)
{
    if(isContainerMode() && impl->isContentsDragEnabled){
        if(impl->dragProjector.drag(event)){
            setPosition(impl->dragProjector.position());
            notifyUpdate();
            impl->sigPositionDragged();
            return true;
        }
    }
    return false;
}


void PositionDragger::onPointerLeaveEvent(const SceneWidgetEvent&)
{
    if(isContainerMode() && impl->isContentsDragEnabled){
        impl->dragProjector.resetDragMode();
    }
}


void PositionDragger::onFocusChanged(const SceneWidgetEvent&, bool on)
{
    if(isContainerMode()){
        impl->showDragMarkers(on || impl->isDraggerAlwaysShown);
    }
}


void PositionDragger::onSceneModeChanged(const SceneWidgetEvent& event)
{
    if(!event.sceneWidget()->isEditMode()){
        impl->showDragMarkers(false);
    }
}


void PositionDragger::storeCurrentPositionToHistory()
{
    impl->storeCurrentPositionToHistory();
}


void PositionDraggerImpl::storeCurrentPositionToHistory()
{
    if(isUndoEnabled){
        history.push_back(self->position());
        if(history.size() > 10){
            history.pop_front();
        }
    }
}


void PositionDragger::setUndoEnabled(bool on)
{
    impl->isUndoEnabled = on;
}


bool PositionDragger::isUndoEnabled() const
{
    return impl->isUndoEnabled;
}


bool PositionDragger::onUndoRequest()
{
    if(!impl->history.empty()){
        const Affine3& T = impl->history.back();
        setPosition(T);
        impl->history.pop_back();
        notifyUpdate();
    }
    return true;
}


bool PositionDragger::onRedoRequest()
{
    return true;
}
