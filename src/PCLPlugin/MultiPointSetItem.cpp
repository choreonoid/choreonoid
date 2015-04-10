/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "MultiPointSetItem.h"
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <boost/bind.hpp>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {

enum RenderingMode {
    POINT_MODE, VOXEL_MODE, N_RENDERING_MODES
};

}

namespace cnoid {
        
class MultiPointSetItemImpl
{
public:
    MultiPointSetItem* self;
    Selection renderingMode;
    double pointSize;
    double voxelSize;
    string directory;

    MultiPointSetItemImpl(MultiPointSetItem* self);
    MultiPointSetItemImpl(MultiPointSetItem* self, const MultiPointSetItemImpl& org);
    void initialize();
    bool setRenderingMode(int mode);
};

}


void MultiPointSetItem::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ItemManager& im = ext->itemManager();
        im.registerClass<MultiPointSetItem>(N_("MultiPointSetItem"));
        im.addCreationPanel<MultiPointSetItem>();
        initialized = true;
    }
}


MultiPointSetItem::MultiPointSetItem()
{
    impl = new MultiPointSetItemImpl(this);
}


MultiPointSetItemImpl::MultiPointSetItemImpl(MultiPointSetItem* self)
    : self(self)
{
    initialize();
    renderingMode.select(POINT_MODE);

}


MultiPointSetItem::MultiPointSetItem(const MultiPointSetItem& org)
    : Item(org)
{
    impl = new MultiPointSetItemImpl(this, *org.impl);
}


MultiPointSetItemImpl::MultiPointSetItemImpl(MultiPointSetItem* self, const MultiPointSetItemImpl& org)
    : self(self)
{
    initialize();
    renderingMode.select(org.renderingMode.which());
}


void MultiPointSetItemImpl::initialize()
{
    renderingMode.setSymbol(POINT_MODE, N_("Point"));
    renderingMode.setSymbol(VOXEL_MODE, N_("Voxel"));
}    


MultiPointSetItem::~MultiPointSetItem()
{
    delete impl;
}


SgNode* MultiPointSetItem::getScene()
{
    return 0;
}


bool MultiPointSetItemImpl::setRenderingMode(int mode)
{
    bool result = true;
    if(mode != renderingMode.which()){
        result = renderingMode.select(mode);
    }
    return result;
}


double MultiPointSetItem::pointSize() const
{
    return impl->pointSize;
}
    

void MultiPointSetItem::setPointSize(double size)
{
    
}


double MultiPointSetItem::voxelSize() const
{
    return impl->voxelSize;
}
    

void MultiPointSetItem::setVoxelSize(double size)
{

}


ItemPtr MultiPointSetItem::doDuplicate() const
{
    return new MultiPointSetItem(*this);
}


void MultiPointSetItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Auto save"), false);
    putProperty(_("Directory"), "");
    putProperty(_("Rendering mode"), impl->renderingMode,
                boost::bind(&MultiPointSetItemImpl::setRenderingMode, impl, _1));
    putProperty.decimals(1).min(0.0)(_("Point size"), pointSize(),
                                     boost::bind(&MultiPointSetItem::setPointSize, this, _1), true);
    putProperty.decimals(4)(_("Voxel size"), voxelSize(),
                            boost::bind(&MultiPointSetItem::setVoxelSize, this, _1), true);
}


bool MultiPointSetItem::store(Archive& archive)
{
    archive.write("autoSave", false);
    if(!impl->directory.empty()){
        archive.writeRelocatablePath("directory", impl->directory);
    }
    archive.write("renderingMode", impl->renderingMode.selectedSymbol());
    archive.write("pointSize", pointSize());
    archive.write("voxelSize", voxelSize());
    return true;
}


bool MultiPointSetItem::restore(const Archive& archive)
{
    archive.get("autoSave", false);
    string directory;
    if(archive.readRelocatablePath("directory", directory)){

    }
    string symbol;
    if(archive.read("renderingMode", symbol)){
        impl->setRenderingMode(impl->renderingMode.index(symbol));
    }
    setPointSize(archive.get("pointSize", pointSize()));
    setVoxelSize(archive.get("voxelSize", voxelSize()));
    
    return true;
}
