/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "PointSetItem.h"
#include <cnoid/ItemManager>
#include <cnoid/Archive>
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

class ScenePointSet : public SgPosTransform, public SceneWidgetEditable
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    bool isEditable_;
    SgPointSetPtr orgPointSet;
    SgPointSetPtr visiblePointSet;
    SgInvariantGroupPtr invariant;
    CrossMarkerPtr clickPointMarker;
    boost::signal<void(const Vector3& point)> sigPointPicked;

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

        if(clickPointMarker){
            removeChild(clickPointMarker);
        }
    }

    virtual bool onButtonPressEvent(const SceneWidgetEvent& event) {

        if(!isEditable_){
            return false;
        }

        if(!clickPointMarker){
            Vector3f color(1.0f, 1.0f, 0.0f);
            clickPointMarker = new CrossMarker(0.01, color);
        }
        clickPointMarker->setTranslation(T().inverse() * event.point());
        addChildOnce(clickPointMarker);
        clickPointMarker->notifyUpdate();

        sigPointPicked(event.point());

        return true;
    }
    
    virtual bool onPointerMoveEvent(const SceneWidgetEvent& event) {
        return false;
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


SgNode* PointSetItem::scene()
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


SignalProxy< boost::signal<void(const Vector3& point)> > PointSetItem::sigPointPicked()
{
    return impl->scenePointSet->sigPointPicked;
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
