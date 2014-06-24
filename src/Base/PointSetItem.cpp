/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "PointSetItem.h"
#include <cnoid/ItemManager>
#include <cnoid/MenuManager>
#include <cnoid/Archive>
#include <cnoid/SceneWidget>
#include <cnoid/SceneWidgetEditable>
#include <cnoid/PointSetUtil>
#include <cnoid/SceneMarker>
#include <cnoid/Exception>
#include <cnoid/FileUtil>
#include <boost/bind.hpp>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class RectLineOverlay : public SgOverlay
{
public:
    SgVertexArrayPtr vertices;
    RectLineOverlay();
    virtual void calcViewVolume(double viewportWidth, double viewportHeight, ViewVolume& io_volume);
    void setRect(int x0, int y0, int x1, int y1);
};

typedef ref_ptr<RectLineOverlay> RectLineOverlayPtr;


class PointSetEraser : public SceneWidgetEditable, public Referenced
{
public:
    SceneWidget* sceneWidget;
    boost::signals::connection connection;
    RectLineOverlayPtr rectLineOverlay;
    int x0, y0;

    PointSetEraser(SceneWidget* sceneWidget);
    void onSceneWidgetAboutToBeDestroyed();
    ~PointSetEraser();

    virtual bool onButtonPressEvent(const SceneWidgetEvent& event);
    virtual bool onButtonReleaseEvent(const SceneWidgetEvent& event);
    virtual bool onPointerMoveEvent(const SceneWidgetEvent& event);
    virtual void onContextMenuRequest(const SceneWidgetEvent& event, MenuManager& menuManager);
};

typedef ref_ptr<PointSetEraser> PointSetEraserPtr;


class ScenePointSet : public SgPosTransform, public SceneWidgetEditable
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    bool isEditable_;
    SgPointSetPtr orgPointSet;
    SgPointSetPtr visiblePointSet;
    SgInvariantGroupPtr invariant;
    boost::optional<Vector3> attentionPoint;
    CrossMarkerPtr attentionPointMarker;
    PointSetEraserPtr eraser;

    ScenePointSet(SgPointSet* pointSet) {
        orgPointSet = pointSet;
        visiblePointSet = new SgPointSet;
        isEditable_ = false;
    }

    bool isEditable() const {
        return isEditable_;
    }

    void setEditable(bool on) {
        isEditable_ = on;
    }

    void setPointSize(double size){
        if(size != visiblePointSet->pointSize()){
            visiblePointSet->setPointSize(size);
            if(invariant){
                updateVisiblePointSet();
            }
        }
    }

    void clearAttentionPoint() {
        attentionPoint = boost::none;
        if(attentionPointMarker){
            removeChild(attentionPointMarker);
            notifyUpdate();
        }
    }

    void updateVisiblePointSet() {
        if(invariant){
            removeChild(invariant);
            invariant->removeChild(visiblePointSet);
        }
        invariant = new SgInvariantGroup;

        //! \todo implement the assignment operator to SgPlot and SgPointSet and use it
        visiblePointSet->setVertices(orgPointSet->vertices());
        visiblePointSet->setNormals(orgPointSet->normals());
        visiblePointSet->normalIndices() = orgPointSet->normalIndices();
        visiblePointSet->setColors(orgPointSet->colors());
        visiblePointSet->colorIndices() = orgPointSet->colorIndices();
    
        invariant->addChild(visiblePointSet);
        addChild(invariant, true);

        if(attentionPointMarker){
            removeChild(attentionPointMarker);
        }
    }

    virtual bool onButtonPressEvent(const SceneWidgetEvent& event) {

        if(!isEditable_){
            return false;
        }

        bool processed = false;
        
        if(event.button() == Qt::LeftButton){
            if(!attentionPointMarker){
                Vector3f color(1.0f, 1.0f, 0.0f);
                attentionPointMarker = new CrossMarker(0.01, color);
            }
            if(attentionPoint && event.point().isApprox(*attentionPoint, 1.0e-3)){
                clearAttentionPoint();
            } else {
                attentionPoint = event.point();
                attentionPointMarker->setTranslation(T().inverse() * *attentionPoint);
                addChildOnce(attentionPointMarker);
                attentionPointMarker->notifyUpdate();
            }
            processed = true;
        }

        return processed;
    }
    
    virtual bool onPointerMoveEvent(const SceneWidgetEvent& event) {
        return false;
    }

    virtual void onContextMenuRequest(const SceneWidgetEvent& event, MenuManager& menuManager) {
        menuManager.addItem(_("Clear Attention Point"))->sigTriggered().connect(
            boost::bind(&ScenePointSet::clearAttentionPoint, this));
        menuManager.addSeparator();
        menuManager.addItem(_("Start Eraser Mode"))->sigTriggered().connect(
            boost::bind(&ScenePointSet::startEraserMode, this, event.sceneWidget()));
    }

    void startEraserMode(SceneWidget* sceneWidget) {
        eraser = new PointSetEraser(sceneWidget);
    }
    
    virtual void onSceneModeChanged(const SceneWidgetEvent& event) {

    }
};

typedef ref_ptr<ScenePointSet> ScenePointSetPtr;

}


namespace cnoid {
        
class PointSetItemImpl
{
public:
    SgPointSetPtr pointSet;
    ScenePointSetPtr scenePointSet;
    boost::signals::connection pointSetUpdateConnection;

    PointSetItemImpl();
    PointSetItemImpl(const PointSetItemImpl& org);
    bool onEditableChanged(bool on);
};

}


static bool loadPCD(PointSetItem* item, const std::string& filename, std::ostream& os)
{
    try {
        cnoid::loadPCD(item->pointSet(), filename);
        os << item->pointSet()->vertices()->size() << " points have been loaded.";
        return true;
    } catch (boost::exception& ex) {
        if(std::string const * message = boost::get_error_info<error_info_message>(ex)){
            os << *message;
        }
    }
    return false;
}


static bool saveAsPCD(PointSetItem* item, const std::string& filename, std::ostream& os)
{
    try {
        cnoid::savePCD(item->pointSet(), filename, item->offsetPosition());
        return true;
    } catch (boost::exception& ex) {
        if(std::string const * message = boost::get_error_info<error_info_message>(ex)){
            os << *message;
        }
    }
    return false;
}


void PointSetItem::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ItemManager& im = ext->itemManager();
        im.registerClass<PointSetItem>(N_("PointSetItem"));
        im.addCreationPanel<PointSetItem>();
        im.addLoaderAndSaver<PointSetItem>(
            _("Point Cloud (PCD)"), "PCD-FILE", "pcd",
            boost::bind(::loadPCD, _1, _2, _3),
            boost::bind(::saveAsPCD, _1, _2, _3),
            ItemManager::PRIORITY_CONVERSION);
        
        initialized = true;
    }
}


PointSetItem::PointSetItem()
{
    impl = new PointSetItemImpl();
    initialize();
}


PointSetItemImpl::PointSetItemImpl()
{
    pointSet = new SgPointSet;
    scenePointSet = new ScenePointSet(pointSet);
}


PointSetItem::PointSetItem(const PointSetItem& org)
    : Item(org)
{
    impl = new PointSetItemImpl(*org.impl);
    initialize();
}


PointSetItemImpl::PointSetItemImpl(const PointSetItemImpl& org)
{
    pointSet = new SgPointSet(*org.pointSet);
    scenePointSet = new ScenePointSet(pointSet);
    scenePointSet->T() = org.scenePointSet->T();
}


void PointSetItem::initialize()
{
    impl->pointSetUpdateConnection = 
        impl->pointSet->sigUpdated().connect(
            boost::bind(&PointSetItem::notifyUpdate, this));
}


PointSetItem::~PointSetItem()
{
    impl->pointSetUpdateConnection.disconnect();
    delete impl;
}


void PointSetItem::setName(const std::string& name)
{
    impl->scenePointSet->setName(name);
    impl->pointSet->setName(name);
    Item::setName(name);
}


SgNode* PointSetItem::getScene()
{
    return impl->scenePointSet;
}


const SgPointSet* PointSetItem::pointSet() const
{
    return impl->pointSet;
}


SgPointSet* PointSetItem::pointSet()
{
    return impl->pointSet;
}


Affine3& PointSetItem::offsetPosition()
{
    return impl->scenePointSet->T();
}


const Affine3& PointSetItem::offsetPosition() const
{
    return impl->scenePointSet->T();
}


void PointSetItem::setPointSize(double size)
{
    impl->scenePointSet->setPointSize(size);
}


double PointSetItem::pointSize() const
{
    return impl->scenePointSet->visiblePointSet->pointSize();
}
    

void PointSetItem::setEditable(bool on)
{
    impl->scenePointSet->setEditable(on);
}


bool PointSetItem::isEditable() const
{
    return impl->scenePointSet->isEditable();
}


bool PointSetItemImpl::onEditableChanged(bool on)
{
    scenePointSet->setEditable(on);
    return true;
}


boost::optional<Vector3> PointSetItem::attentionPoint() const
{
    return impl->scenePointSet->attentionPoint;
}


void PointSetItem::notifyUpdate()
{
    impl->scenePointSet->updateVisiblePointSet();
    Item::notifyUpdate();
}


ItemPtr PointSetItem::doDuplicate() const
{
    return new PointSetItem(*this);
}


void PointSetItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("File"), getFilename(filePath()));
    putProperty.decimals(1).min(0.0)(_("Point size"), pointSize(),
                                     boost::bind(&PointSetItem::setPointSize, this, _1), true);
    putProperty(_("Editable"), isEditable(), boost::bind(&PointSetItemImpl::onEditableChanged, impl, _1));
}


bool PointSetItem::store(Archive& archive)
{
    if(!filePath().empty()){
        archive.writeRelocatablePath("file", filePath());
        archive.write("format", fileFormat());
    }
    archive.write("pointSize", pointSize());
    archive.write("isEditable", isEditable());
    return true;
}


bool PointSetItem::restore(const Archive& archive)
{
    setPointSize(archive.get("pointSize", 0.0));
    setEditable(archive.get("isEditable", isEditable()));
    
    std::string filename, formatId;
    if(archive.readRelocatablePath("file", filename) && archive.read("format", formatId)){
        if(load(filename, archive.currentParentItem(), formatId)){
            return true;
        }
    }
    return true;
}


PointSetEraser::PointSetEraser(SceneWidget* sceneWidget)
  : sceneWidget(sceneWidget)
{
    rectLineOverlay = new RectLineOverlay;
    
    sceneWidget->installEventFilter(this);
    connection = sceneWidget->sigAboutToBeDestroyed().connect(
        boost::bind(&PointSetEraser::onSceneWidgetAboutToBeDestroyed, this));
}


void PointSetEraser::onSceneWidgetAboutToBeDestroyed()
{
    sceneWidget = 0;
}


PointSetEraser::~PointSetEraser()
{
    if(sceneWidget){
        if(rectLineOverlay->hasParents()){
            sceneWidget->sceneRoot()->removeChild(rectLineOverlay, true);
        }
        sceneWidget->removeEventFilter(this);
        connection.disconnect();
    }
}


bool PointSetEraser::onButtonPressEvent(const SceneWidgetEvent& event)
{
    x0 = event.x();
    y0 = event.y();
    rectLineOverlay->setRect(x0, y0, x0, y0);
    
    if(!rectLineOverlay->hasParents()){
        sceneWidget->sceneRoot()->addChild(rectLineOverlay, true);
    }
}


bool PointSetEraser::onButtonReleaseEvent(const SceneWidgetEvent& event)
{

}


bool PointSetEraser::onPointerMoveEvent(const SceneWidgetEvent& event)
{
    rectLineOverlay->setRect(x0, y0, event.x(), event.y());
}


void PointSetEraser::onContextMenuRequest(const SceneWidgetEvent& event, MenuManager& menuManager)
{

}


RectLineOverlay::RectLineOverlay()
{
    SgLineSet* lineSet = new SgLineSet;
    vertices = lineSet->getOrCreateVertices();
    vertices->resize(4);
    lineSet->getOrCreateColors()->push_back(Vector3f(1.0f, 0.0f, 0.0f));
    lineSet->reserveNumLines(4);
    SgIndexArray& colorIndices = lineSet->colorIndices();
    colorIndices.reserve(8);
    for(int i=0; i < 4; ++i){
        lineSet->addLine(i, (i + 1) % 4);
        colorIndices.push_back(0);
        colorIndices.push_back(0);
    }
    addChild(lineSet);
}


void RectLineOverlay::calcViewVolume(double viewportWidth, double viewportHeight, ViewVolume& io_volume)
{
    io_volume.left = 0;
    io_volume.right = viewportWidth;
    io_volume.bottom = 0;
    io_volume.top = viewportHeight;
}


void RectLineOverlay::setRect(int x0, int y0, int x1, int y1)
{
    float left, right, top, bottom;

    if(x0 <= x1){
        left = x0;
        right = x1;
    } else {
        left = x1;
        right = x0;
    }
    if(y0 >= y1){
        top = y0;
        bottom = y1;
    } else {
        top = y1;
        bottom = y0;
    }

    vertices->at(0) << left,  top,    0.0f;
    vertices->at(1) << left,  bottom, 0.0f;
    vertices->at(2) << right, bottom, 0.0f;
    vertices->at(3) << right, top,    0.0f;

    vertices->notifyUpdate();
}
