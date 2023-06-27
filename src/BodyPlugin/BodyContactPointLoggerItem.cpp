#include "BodyContactPointLoggerItem.h"
#include <cnoid/ItemManager>
#include <cnoid/SceneDrawables>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class BodyContactPointLoggerItem::Impl
{
public:
    BodyContactPointLoggerItem* self;
    ControllerIO* io;
    Body* ioBody;
    BodyContactPointLogItem::LogFramePtr logFrameToVisualize;
    bool isChecked;
    SgLineSetPtr contactLineSet;
    SgUpdate update;

    Impl(BodyContactPointLoggerItem* self);
    void onCheckToggled();
    void updateContactLineSet();
};

}


void BodyContactPointLoggerItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager()
        .registerClass<BodyContactPointLoggerItem, ControllerItem>(N_("BodyContactPointLoggerItem"))
        .addCreationPanel<BodyContactPointLoggerItem>();
}


BodyContactPointLoggerItem::BodyContactPointLoggerItem()
{
    impl = new Impl(this);
}


BodyContactPointLoggerItem::BodyContactPointLoggerItem(const BodyContactPointLoggerItem& org)
    : ControllerItem(org)
{
    impl = new Impl(this);
}


Item* BodyContactPointLoggerItem::doCloneItem(CloneMap* /* cloneMap */) const
{
    return new BodyContactPointLoggerItem(*this);
}


BodyContactPointLoggerItem::Impl::Impl(BodyContactPointLoggerItem* self)
    : self(self)
{
    self->sigCheckToggled(LogicalSumOfAllChecks).connect(
        [this](bool){ onCheckToggled(); });
    isChecked = false;
}


bool BodyContactPointLoggerItem::initialize(ControllerIO* io)
{
    impl->io = io;
    impl->ioBody = io->body();
    
    for(auto& link : impl->ioBody->links()){
        link->mergeSensingMode(Link::LinkContactState);
    }
    
    return io->enableLog();
}


ReferencedObjectSeqItem* BodyContactPointLoggerItem::createLogItem()
{
    return new BodyContactPointLogItem;
}


void BodyContactPointLoggerItem::outputLogFrame()
{
    auto logFrame = new BodyContactPointLogItem::LogFrame;
    int numLinks = impl->ioBody->numLinks();
    logFrame->bodyContactPoints().resize(numLinks);
    for(int i=0; i < numLinks; ++i){
        logFrame->linkContactPoints(i) = impl->ioBody->link(i)->contactPoints();
    }
    impl->io->outputLogFrame(logFrame);
}


void BodyContactPointLoggerItem::Impl::onCheckToggled()
{
    bool on = self->isChecked(LogicalSumOfAllChecks);
    if(on != isChecked){
        if(on && contactLineSet){
            updateContactLineSet();
        }
        isChecked = on;
    }
}


SgNode* BodyContactPointLoggerItem::getScene()
{
    if(!impl->contactLineSet){
        impl->contactLineSet = new SgLineSet;
        impl->updateContactLineSet();
    }
    return impl->contactLineSet;
}


void BodyContactPointLoggerItem::setLogFrameToVisualize(BodyContactPointLogItem::LogFrame* logFrame)
{
    impl->logFrameToVisualize = logFrame;
    
    if(!impl->contactLineSet){
        return;
    }

    impl->updateContactLineSet();
}


void BodyContactPointLoggerItem::Impl::updateContactLineSet()
{
    contactLineSet->clearLines();

    if(logFrameToVisualize){
        auto vertices = contactLineSet->getOrCreateVertices();
        vertices->clear();
        int vertexIndex = 0;
        for(auto& points : logFrameToVisualize->bodyContactPoints()){
            for(auto& point : points){
                auto p = point.position().cast<Vector3f::Scalar>();
                auto f = 1.0e-3f * point.force().cast<Vector3f::Scalar>();
                vertices->push_back(p);
                vertices->push_back(p + f);
                contactLineSet->addLine(vertexIndex, vertexIndex + 1);
                vertexIndex += 2;
            }
        }
    }
    
    contactLineSet->notifyUpdate(update);
}
