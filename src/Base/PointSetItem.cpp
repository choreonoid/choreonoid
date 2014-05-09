/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "PointSetItem.h"
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/PointSetUtil>
#include <cnoid/Exception>
#include <cnoid/FileUtil>
#include <boost/bind.hpp>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {
        
bool loadPCD(PointSetItem* item, const std::string& filename, std::ostream& os)
{
    try {
        SgPointSet* pointSet = item->pointSet();
        cnoid::loadPCD(pointSet, filename);
        os << pointSet->vertices()->size() << " points have been loaded.";
        return true;
    } catch (boost::exception& ex) {
        if(std::string const * message = get_error_info<error_info_message>(ex)){
            os << *message;
        }
    }
    return false;
}

bool saveAsPCD(PointSetItem* item, const std::string& filename, std::ostream& os)
{
    try {
        cnoid::savePCD(item->pointSet(), filename, item->offsetPosition());
        return true;
    } catch (boost::exception& ex) {
        if(std::string const * message = get_error_info<error_info_message>(ex)){
            os << *message;
        }
    }
    return false;
}
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
            bind(::loadPCD, _1, _2, _3), bind(::saveAsPCD, _1, _2, _3),
            ItemManager::PRIORITY_CONVERSION);
        
        initialized = true;
    }
}


PointSetItem::PointSetItem()
{
    topTransform = new SgPosTransform;
    pointSet_ = new SgPointSet;
    initMembers();
}


PointSetItem::PointSetItem(const PointSetItem& org)
    : Item(org)
{
    topTransform = new SgPosTransform(*org.topTransform);
    pointSet_ = new SgPointSet(*org.pointSet_);
    initMembers();
}


void PointSetItem::initMembers()
{
    visiblePointSet = new SgPointSet;
    pointSet_->sigUpdated().connect(bind(&PointSetItem::updateVisiblePointSet, this));
}


PointSetItem::~PointSetItem()
{
    
}


void PointSetItem::setName(const std::string& name)
{
    topTransform->setName(name);
    pointSet_->setName(name);
    Item::setName(name);
}


SgNode* PointSetItem::scene()
{
    return topTransform;
}


void PointSetItem::setPointSize(double size)
{
    if(size != visiblePointSet->pointSize()){
        visiblePointSet->setPointSize(size);
        if(invariant){
            updateVisiblePointSet();
        }
    }
}


void PointSetItem::notifyUpdate()
{
    updateVisiblePointSet();
}


void PointSetItem::updateVisiblePointSet()
{
    topTransform->removeChild(invariant);
    invariant = new SgInvariantGroup;

    //! \todo implement the assignment operator to SgPlot and SgPointSet and use it
    visiblePointSet->setVertices(pointSet_->vertices());
    visiblePointSet->setNormals(pointSet_->normals());
    visiblePointSet->normalIndices() = pointSet_->normalIndices();
    visiblePointSet->setColors(pointSet_->colors());
    visiblePointSet->colorIndices() = pointSet_->colorIndices();
    
    invariant->addChild(visiblePointSet);
    topTransform->addChild(invariant, true);

    Item::notifyUpdate();    
}


ItemPtr PointSetItem::doDuplicate() const
{
    return new PointSetItem(*this);
}


void PointSetItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("File"), getFilename(filePath()));
    putProperty.decimals(1).min(0.0)(_("Point size"), visiblePointSet->pointSize(),
                                     bind(&PointSetItem::setPointSize, this, _1), true);
}


bool PointSetItem::store(Archive& archive)
{
    if(!filePath().empty()){
        archive.writeRelocatablePath("file", filePath());
        archive.write("format", fileFormat());
    }
    archive.write("pointSize", visiblePointSet->pointSize());
    return true;
}


bool PointSetItem::restore(const Archive& archive)
{
    setPointSize(archive.get("pointSize", 0.0));
    
    std::string filename, formatId;
    if(archive.readRelocatablePath("file", filename) && archive.read("format", formatId)){
        if(load(filename, archive.currentParentItem(), formatId)){
            return true;
        }
    }
    return true;
}
